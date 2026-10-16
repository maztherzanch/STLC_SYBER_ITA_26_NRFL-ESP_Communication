/*
  NODO TELEFONO v6 (ID Automatico por Central + Audio Fluido)
  Compatible con Central_Estrella.ino v6 y NodoTelefonoInterfaz.html.
  No requiere configurar MI_ID manualmente: la central asigna el ID segun la MAC del ESP32.
*/
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#if defined(CONFIG_IDF_TARGET_ESP32C3)
  #define PIN_CE    3
  #define PIN_CSN   7
  #define PIN_SCK   4
  #define PIN_MISO  5
  #define PIN_MOSI  6
  #define PIN_LED   8
  #define PIN_BOOT  9
  #define LED_ON    LOW
  #define PLACA     "ESP32-C3 SuperMini"
#elif defined(CONFIG_IDF_TARGET_ESP32)
  #define PIN_CE    2
  #define PIN_CSN   4
  #define PIN_SCK   18
  #define PIN_MISO  19
  #define PIN_MOSI  23
  #define PIN_LED   12  // Si el LED impide bootear en tu DevKit, cambiar a 25
  #define PIN_BOOT  0
  #define LED_ON    HIGH
  #define PLACA     "ESP32 DevKit"
#else
  #error "Placa no soportada"
#endif
#define LED_OFF (LED_ON == HIGH ? LOW : HIGH)

#define VELOCIDAD_SERIE    460800
#define ID_CENTRAL         0
#define ID_SIN_ASIGNAR     255
#define CANAL_CONTROL      80
#define CUERPO             29

// Timeouts de protocolo
#define PEDIR_ID_CADA_MS   1500UL
#define REGISTRO_CADA_MS   5000UL
#define TIMEOUT_TIMBRE_MS  25000UL
#define KEEPALIVE_CADA_MS  2000UL
#define TIMEOUT_LLAMADA_MS 8000UL

enum Tipo : uint8_t {
  REGISTRO = 1, LLAMAR, CONTESTAR, RECHAZAR, LIBERAR,
  ENTRANTE, CONECTAR, RESPUESTA, TEXTO, KEEPALIVE, COLGAR,
  AVISO, VOZ, AUDIO, PEDIR_ID, ASIGNAR_ID
};
enum Codigo : uint8_t {
  R_TIMBRANDO = 1, R_OCUPADO, R_NO_DISPONIBLE, R_RECHAZADO,
  R_NO_CONTESTA, R_SIN_CANALES, R_CANCELADO
};
struct Paquete {
  uint8_t tipo;
  uint8_t origen;
  uint8_t destino;
  uint8_t canal;
  uint8_t dato;
  uint8_t secuencia;
  char    texto[26];
};
static_assert(sizeof(Paquete) == 32, "El paquete debe medir 32 bytes");

enum Estado { REPOSO, LLAMANDO, TIMBRANDO, EN_LLAMADA };

RF24 radio(PIN_CE, PIN_CSN);
uint8_t miId = 0;                // 0 = esperando asignacion de la central
char miMac[13] = {};             // MAC unica del ESP32 en formato hexadecimal
uint8_t secuencia = 0;

Estado estado = REPOSO;
uint8_t pareja = 0;
uint8_t canalActual = CANAL_CONTROL;
uint8_t ultimoDestinoPipe = 254;

unsigned long tEstado = 0;
unsigned long tUltimoEnvio = 0;
unsigned long tUltimaRxPar = 0;
unsigned long tRegistro = 0;
bool centralPerdida = false;

bool audioPC = false;
uint8_t nivelRemoto = 0;
unsigned long tUltimoAudioRx = 0;

// Cola circular de tramas de voz para evitar saturar el radio en modo half-duplex
#define COLA_VOZ_TAM 16
uint8_t colaVoz[COLA_VOZ_TAM][CUERPO];
uint8_t colaHead = 0, colaTail = 0, colaCount = 0;
unsigned long tUltimoVozTxUs = 0;

const char *nombreEstado[] = {"REPOSO", "LLAMANDO", "TIMBRANDO", "EN LLAMADA"};

void direccionDe(uint8_t id, uint8_t *dir) {
  dir[0] = id; dir[1] = 'N'; dir[2] = 'O'; dir[3] = 'D'; dir[4] = 'E';
}

void direccionTempDeMac(const char *mac, uint8_t *dir) {
  dir[0] = ID_SIN_ASIGNAR;
  dir[1] = mac[8]  ? (uint8_t)mac[8]  : '0';
  dir[2] = mac[9]  ? (uint8_t)mac[9]  : '0';
  dir[3] = mac[10] ? (uint8_t)mac[10] : '0';
  dir[4] = mac[11] ? (uint8_t)mac[11] : '0';
}

void actualizarPipeLectura() {
  uint8_t dir[5];
  if (miId == 0) direccionTempDeMac(miMac, dir);
  else           direccionDe(miId, dir);
  radio.openReadingPipe(1, dir);
}

void fijarPipeEscritura(uint8_t destino) {
  if (ultimoDestinoPipe == destino) return;
  uint8_t dir[5];
  direccionDe(destino, dir);
  radio.openWritingPipe(dir);
  ultimoDestinoPipe = destino;
}

bool enviar(uint8_t destino, uint8_t tipo, uint8_t dato, const char *texto, uint8_t intentos = 1) {
  Paquete p = {};
  p.tipo = tipo;
  p.origen = (miId == 0) ? ID_SIN_ASIGNAR : miId;
  p.destino = destino;
  p.canal = canalActual;
  p.dato = dato;
  p.secuencia = secuencia++;
  if (texto) strncpy(p.texto, texto, sizeof(p.texto) - 1);

  radio.stopListening();
  fijarPipeEscritura(destino);
  bool ok = false;
  for (uint8_t k = 0; k < intentos; k++) {
    if (radio.write(&p, sizeof(p))) { ok = true; break; }
    if (k + 1 < intentos) delayMicroseconds(1200);
  }
  if (estado == EN_LLAMADA && pareja != 0) {
    fijarPipeEscritura(pareja);
  }
  radio.startListening();
  tUltimoEnvio = millis();
  return ok;
}

void encolarVozTx(const uint8_t *cuerpo) {
  if (colaCount >= COLA_VOZ_TAM) {
    colaTail = (colaTail + 1) % COLA_VOZ_TAM;
    colaCount--;
  }
  memcpy(colaVoz[colaHead], cuerpo, CUERPO);
  colaHead = (colaHead + 1) % COLA_VOZ_TAM;
  colaCount++;
}

void transmitirVozDeCola() {
  if (colaCount == 0 || estado != EN_LLAMADA || miId == 0) return;
  unsigned long ahoraUs = micros();
  unsigned long intervaloUs = 4800UL + (miId % 3) * 250UL;
  if (ahoraUs - tUltimoVozTxUs < intervaloUs) return;
  tUltimoVozTxUs = ahoraUs;

  uint8_t b[32];
  b[0] = VOZ; b[1] = miId; b[2] = secuencia++;
  memcpy(b + 3, colaVoz[colaTail], CUERPO);
  colaTail = (colaTail + 1) % COLA_VOZ_TAM;
  colaCount--;

  radio.stopListening();
  fijarPipeEscritura(pareja);
  radio.write(b, 32, true);
  radio.startListening();
  tUltimoEnvio = millis();
}

void vozAlPC(const uint8_t *cuerpo) {
  uint8_t t[2 + CUERPO];
  t[0] = 0xFE; t[1] = 0xFF;
  memcpy(t + 2, cuerpo, CUERPO);
  Serial.write(t, sizeof(t));
}

void cambiarCanal(uint8_t c) {
  radio.stopListening();
  radio.setChannel(c);
  radio.startListening();
  canalActual = c;
}

void cambiarEstado(Estado e) {
  estado = e;
  tEstado = millis();
  tUltimaRxPar = millis();
  nivelRemoto = 0;
  colaHead = colaTail = colaCount = 0;
}

void ledBrillo(uint8_t v) {
  analogWrite(PIN_LED, LED_ON == HIGH ? v : 255 - v);
}

uint8_t gamma8(uint8_t v) { return (uint16_t)v * v / 255; }

void volverAReposo(bool avisarCentral) {
  if (canalActual != CANAL_CONTROL) cambiarCanal(CANAL_CONTROL);
  if (avisarCentral && miId != 0) enviar(ID_CENTRAL, LIBERAR, 0, nullptr, 2);
  pareja = 0;
  cambiarEstado(REPOSO);
  if (miId != 0) enviar(ID_CENTRAL, REGISTRO, 0, miMac);
  tRegistro = millis();
  Serial.println("En reposo.");
}

void procesarLinea(String l) {
  l.trim();
  if (l.length() == 0) return;

  if (l.startsWith("/audio")) {
    audioPC = l.substring(6).toInt() != 0;
    return;
  }

  if (l == "/estado") {
    if (miId == 0) {
      Serial.printf("Esperando ID de la central (MAC %s)...\n", miMac);
    } else if (pareja) {
      Serial.printf("Nodo %u | %s | canal %u | con nodo %u\n", miId, nombreEstado[estado], canalActual, pareja);
    } else {
      Serial.printf("Nodo %u | %s | canal %u |\n", miId, nombreEstado[estado], canalActual);
    }
    return;
  }

  if (miId == 0) {
    Serial.println("[ERR] Todavia sin ID asignado por la central.");
    return;
  }

  if (l.startsWith("/llamar")) {
    if (estado != REPOSO) { Serial.println("[ERR] Ocupado"); return; }
    int destino = l.substring(7).toInt();
    if (destino < 1 || destino > 254 || destino == miId) return;
    if (!enviar(ID_CENTRAL, LLAMAR, destino, nullptr, 2)) {
      Serial.println("[ERR] La central no responde, no se pudo llamar");
      return;
    }
    pareja = destino;
    cambiarEstado(LLAMANDO);
    Serial.printf("Llamando al nodo %u\n", pareja);
  }
  else if (l == "/contestar") {
    if (estado != TIMBRANDO) return;
    enviar(ID_CENTRAL, CONTESTAR, 0, nullptr, 2);
  }
  else if (l == "/rechazar") {
    if (estado != TIMBRANDO) return;
    enviar(ID_CENTRAL, RECHAZAR, 0, nullptr, 2);
    volverAReposo(false);
  }
  else if (l == "/colgar") {
    if (estado == EN_LLAMADA) {
      enviar(pareja, COLGAR, 0, nullptr, 3);
      volverAReposo(true);
    } else if (estado == LLAMANDO || estado == TIMBRANDO) {
      volverAReposo(true);
    }
  }
  else {
    if (estado != EN_LLAMADA) return;
    String t = l.substring(0, 25);
    enviar(pareja, TEXTO, 0, t.c_str(), 2);
    Serial.printf("[YO] %s\n", t.c_str());
  }
}

void procesarPaquete(Paquete &p) {
  p.texto[sizeof(p.texto) - 1] = '\0';

  if (p.origen == ID_CENTRAL) {
    centralPerdida = false;
    switch (p.tipo) {
      case ASIGNAR_ID:
        if (strncmp(p.texto, miMac, 12) == 0 && p.dato >= 1 && p.dato < ID_SIN_ASIGNAR) {
          miId = p.dato;
          radio.stopListening();
          actualizarPipeLectura();
          radio.setRetries(3 + (miId % 10), 15);
          radio.startListening();
          cambiarEstado(REPOSO);
          tRegistro = millis();
          Serial.printf("Nodo telefono %u listo (%s - MAC %s)\n", miId, PLACA, miMac);
        }
        break;
      case AVISO:
        Serial.printf("[CENTRAL] %s\n", p.texto);
        break;
      case ENTRANTE:
        if (miId == 0 || estado != REPOSO) break;
        pareja = p.dato;
        cambiarEstado(TIMBRANDO);
        Serial.printf("El nodo %u te llama\n", pareja);
        break;
      case CONECTAR:
        if (miId == 0 || (estado != LLAMANDO && estado != TIMBRANDO)) break;
        pareja = p.dato;
        cambiarCanal(p.canal);
        radio.stopListening();
        fijarPipeEscritura(pareja);
        radio.startListening();
        cambiarEstado(EN_LLAMADA);
        Serial.printf("CONECTADO con nodo %u (canal %u)\n", pareja, p.canal);
        break;
      case RESPUESTA:
        if (p.dato != R_TIMBRANDO) { volverAReposo(false); }
        break;
      case COLGAR:
        if (estado != REPOSO) { volverAReposo(false); }
        break;
    }
    return;
  }

  if (miId == 0 || estado != EN_LLAMADA || p.origen != pareja) return;
  tUltimaRxPar = millis();

  switch (p.tipo) {
    case TEXTO:
      Serial.printf("[MSG %u] %s\n", p.origen, p.texto);
      break;
    case KEEPALIVE:
      break;
    case COLGAR:
      volverAReposo(true);
      break;
  }
}

void leerSerie() {
  static uint8_t fase = 0, pos = 0;
  static uint8_t cuerpo[CUERPO];
  static String linea = "";
  static unsigned long tInicioTrama = 0;

  if (fase != 0 && millis() - tInicioTrama > 15) {
    fase = 0;
    pos = 0;
  }

  uint8_t leidos = 0;
  while (Serial.available() && leidos < 64) {
    uint8_t c = Serial.read();
    leidos++;
    if (fase == 2) {
      cuerpo[pos++] = c;
      if (pos == CUERPO) {
        fase = 0;
        if (estado == EN_LLAMADA && miId != 0) encolarVozTx(cuerpo);
      }
      continue;
    }
    if (fase == 1) {
      if (c == 0xFF) { fase = 2; pos = 0; continue; }
      if (c == 0xFE) { tInicioTrama = millis(); continue; }
      fase = 0;
    }
    if (c == 0xFE) {
      fase = 1;
      tInicioTrama = millis();
    }
    else if (c == '\n' || c == '\r') {
      if (linea.length() > 0) { procesarLinea(linea); linea = ""; }
    }
    else if (linea.length() < 100) {
      linea += (char)c;
    }
  }
}

void configurarRadio() {
  uint64_t chipMac = ESP.getEfuseMac();
  radio.setChannel(canalActual);
  radio.setDataRate(RF24_1MBPS);
  radio.setPALevel(RF24_PA_LOW);
  radio.setPayloadSize(sizeof(Paquete));
  radio.setAutoAck(true);
  radio.enableDynamicAck();
  uint8_t semilla = (miId != 0) ? miId : (uint8_t)(chipMac & 0xFF);
  radio.setRetries(3 + (semilla % 10), 15);
  actualizarPipeLectura();
  ultimoDestinoPipe = 254;
  if (estado == EN_LLAMADA && pareja != 0) fijarPipeEscritura(pareja);
  radio.startListening();
}

void revisarRadio() {
  static unsigned long t = 0;
  if (millis() - t < 500) return;
  t = millis();
  if (radio.getChannel() != canalActual) {
    radio.begin(&SPI);
    configurarRadio();
  }
}

void setup() {
  Serial.setRxBufferSize(2048);
  Serial.begin(VELOCIDAD_SERIE);
#if ARDUINO_USB_CDC_ON_BOOT
  Serial.setTxTimeoutMs(0);
#endif
  delay(1500);
  pinMode(PIN_LED, OUTPUT);
  ledBrillo(0);
  pinMode(PIN_BOOT, INPUT_PULLUP);

  // Generar cadena hexadecimal unica de 12 caracteres a partir de la MAC del ESP32
  uint64_t mac = ESP.getEfuseMac();
  snprintf(miMac, sizeof(miMac), "%04X%08X", (uint16_t)(mac >> 32), (uint32_t)mac);

  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CSN);
  if (!radio.begin(&SPI)) {
    while (true) { ledBrillo(255); delay(100); ledBrillo(0); delay(100); }
  }
  configurarRadio();

  tRegistro = millis() - PEDIR_ID_CADA_MS;
  Serial.printf("Esperando ID de la central (%s - MAC %s)...\n", PLACA, miMac);
}

void loop() {
  // 1) Priorizar siempre la recepcion de radio
  while (radio.available()) {
    uint8_t buf[32];
    radio.read(buf, sizeof(buf));
    if (buf[0] == VOZ) {
      if (miId != 0 && estado == EN_LLAMADA && buf[1] == pareja) {
        tUltimaRxPar = millis();
        tUltimoAudioRx = millis();
        nivelRemoto = constrain(buf[3] * 3, 0, 255);
        if (audioPC) vozAlPC(buf + 3);
      }
    } else {
      Paquete p;
      memcpy(&p, buf, sizeof(p));
      procesarPaquete(p);
    }
  }

  // 2) Leer puerto serie y transmitir audio de la cola con espaciado
  leerSerie();
  transmitirVozDeCola();

  unsigned long ahora = millis();

  if (estado == EN_LLAMADA && ahora - tUltimoAudioRx > 120) {
    nivelRemoto = 0;
  }

  // 3) Si aun no tenemos ID, solicitarlo a la central periodicamente
  if (miId == 0) {
    unsigned long intervaloId = PEDIR_ID_CADA_MS + (miMac[11] % 7) * 80UL;
    if (ahora - tRegistro >= intervaloId) {
      tRegistro = ahora;
      enviar(ID_CENTRAL, PEDIR_ID, 0, miMac);
    }
  }
  // 4) Gestion normal de estados cuando ya tenemos ID
  else if (estado == REPOSO) {
    if (ahora - tRegistro >= REGISTRO_CADA_MS) {
      tRegistro = ahora;
      bool ok = enviar(ID_CENTRAL, REGISTRO, 0, miMac);
      if (ok) centralPerdida = false;
      else if (!centralPerdida) {
        centralPerdida = true;
        Serial.println("[ERR] La central no responde");
      }
    }
  }
  else if (estado == LLAMANDO || estado == TIMBRANDO) {
    if (ahora - tEstado >= TIMEOUT_TIMBRE_MS) {
      Serial.println("Cancelando por tiempo de espera.");
      volverAReposo(true);
    }
  }
  else if (estado == EN_LLAMADA) {
    if (ahora - tUltimoEnvio >= KEEPALIVE_CADA_MS) {
      enviar(pareja, KEEPALIVE, 0, nullptr);
    }
    if (ahora - tUltimaRxPar >= TIMEOUT_LLAMADA_MS) {
      Serial.println("Llamada terminada (sin senal del otro nodo).");
      volverAReposo(true);
    }
  }

  // 5) Salud del radio y LED indicador
  revisarRadio();
  uint8_t brillo = 0;
  if (miId == 0)            brillo = ((ahora / 120) % 8 == 0) ? 180 : 0; // Destello corto esperando ID
  else if (estado == TIMBRANDO)  brillo = ((ahora / 150) % 2) ? 255 : 0;
  else if (estado == LLAMANDO)   brillo = ((ahora / 600) % 2) ? 255 : 0;
  else if (estado == EN_LLAMADA) brillo = max((uint8_t)6, gamma8(nivelRemoto));
  ledBrillo(brillo);
}