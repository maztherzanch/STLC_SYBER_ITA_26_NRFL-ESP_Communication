/*
  NODO TELEFONO - Topologia estrella con central (nRF24L01+ PA+LNA + ESP32)
  Version para la interfaz HTML (Web Serial, "Audio ADPCM binario", 460800 baudios).

  La PC manda la voz ya comprimida (ADPCM 4 bits, 8 kHz) en tramas por el puerto serie:
      0xFE 0xFF + 29 bytes   (1 byte indice + 2 bytes predictor + 26 bytes de 52 muestras)
  El ESP32 solo la reenvia por radio al otro nodo y hace lo mismo en sentido contrario.
  Cada paquete de voz mide 32 bytes: [VOZ][origen][secuencia][29 bytes de audio]
*/
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define MI_ID 1         // <-- distinto en cada nodo: 1, 2, 3 ...

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
  #define PIN_LED   12
  #define PIN_BOOT  0
  #define LED_ON    HIGH
  #define PLACA     "ESP32 DevKit"
#else
  #error "Placa no soportada"
#endif
#define LED_OFF (LED_ON == HIGH ? LOW : HIGH)

#define VELOCIDAD_SERIE 460800   // debe coincidir con la velocidad elegida en el HTML

#define ID_CENTRAL     0
#define CANAL_CONTROL  80
#define CUERPO         29        // bytes de audio por paquete

enum Tipo : uint8_t {
  REGISTRO = 1, LLAMAR, CONTESTAR, RECHAZAR, LIBERAR,
  ENTRANTE, CONECTAR, RESPUESTA, TEXTO, KEEPALIVE, COLGAR,
  AVISO, VOZ
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

void direccionDe(uint8_t id, uint8_t *dir) {
  dir[0] = id; dir[1] = 'N'; dir[2] = 'O'; dir[3] = 'D'; dir[4] = 'E';
}

RF24 radio(PIN_CE, PIN_CSN);
uint8_t secuencia = 0;

Estado estado = REPOSO;
uint8_t pareja = 0;
uint8_t canalActual = CANAL_CONTROL;
unsigned long tEstado = 0;
unsigned long tUltimoEnvio = 0;
unsigned long tRegistro = 0;
bool centralPerdida = false;

bool audioPC = false;            // la PC pidio recibir audio (/audio 1)
uint8_t nivelRemoto = 0;         // solo para el brillo del LED

#define REGISTRO_CADA_MS   5000UL

const char *nombreEstado[] = {"REPOSO", "LLAMANDO", "TIMBRANDO", "EN LLAMADA"};

bool enviar(uint8_t destino, uint8_t tipo, uint8_t dato, const char *texto) {
  Paquete p = {};
  p.tipo = tipo; p.origen = MI_ID; p.destino = destino;
  p.canal = canalActual; p.dato = dato; p.secuencia = secuencia++;
  if (texto) strncpy(p.texto, texto, sizeof(p.texto) - 1);
  uint8_t dir[5]; direccionDe(destino, dir);
  radio.stopListening();
  radio.openWritingPipe(dir);
  bool ok = radio.write(&p, sizeof(p));
  radio.startListening();
  tUltimoEnvio = millis();
  return ok;
}

// Voz hacia el otro nodo: sin ACK, si se pierde un paquete no se reintenta
// (cada paquete lleva su propio estado ADPCM, asi que el decodificador se recupera solo).
void enviarVoz(const uint8_t *cuerpo) {
  uint8_t b[32];
  b[0] = VOZ; b[1] = MI_ID; b[2] = secuencia++;
  memcpy(b + 3, cuerpo, CUERPO);
  uint8_t dir[5]; direccionDe(pareja, dir);
  radio.stopListening();
  radio.openWritingPipe(dir);
  radio.write(b, 32, true);
  radio.startListening();
}

// Voz hacia la PC: trama binaria 0xFE 0xFF + 29 bytes, escrita de una sola vez
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
  nivelRemoto = 0;
}

void ledBrillo(uint8_t v) {
  analogWrite(PIN_LED, LED_ON == HIGH ? v : 255 - v);
}

uint8_t gamma8(uint8_t v) { return (uint16_t)v * v / 255; }

void volverAReposo(bool avisarCentral) {
  if (canalActual != CANAL_CONTROL) cambiarCanal(CANAL_CONTROL);
  if (avisarCentral) enviar(ID_CENTRAL, LIBERAR, 0, nullptr);
  pareja = 0;
  cambiarEstado(REPOSO);
  enviar(ID_CENTRAL, REGISTRO, 0, nullptr);
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

  if (l.startsWith("/llamar")) {
    if (estado != REPOSO) { Serial.println("[ERR] Ocupado"); return; }
    int destino = l.substring(7).toInt();
    if (destino < 1 || destino > 254 || destino == MI_ID) return;
    if (!enviar(ID_CENTRAL, LLAMAR, destino, nullptr)) {
      Serial.println("[ERR] La central no responde, no se pudo llamar");
      return;
    }
    pareja = destino;
    cambiarEstado(LLAMANDO);
    Serial.printf("Llamando al nodo %u\n", pareja);
  }
  else if (l == "/contestar") {
    if (estado != TIMBRANDO) return;
    enviar(ID_CENTRAL, CONTESTAR, 0, nullptr);
  }
  else if (l == "/rechazar") {
    if (estado != TIMBRANDO) return;
    enviar(ID_CENTRAL, RECHAZAR, 0, nullptr);
    volverAReposo(false);
  }
  else if (l == "/colgar") {
    if (estado == EN_LLAMADA) {
      enviar(pareja, COLGAR, 0, nullptr);
      volverAReposo(true);
    } else if (estado == LLAMANDO || estado == TIMBRANDO) {
      volverAReposo(true);
    }
  }
  else if (l == "/estado") {
    if (pareja) Serial.printf("Nodo %u | %s | canal %u | con nodo %u\n", MI_ID, nombreEstado[estado], canalActual, pareja);
    else        Serial.printf("Nodo %u | %s | canal %u |\n", MI_ID, nombreEstado[estado], canalActual);
  }
  else {
    if (estado != EN_LLAMADA) return;
    String t = l.substring(0, 25);
    enviar(pareja, TEXTO, 0, t.c_str());
    Serial.printf("[YO] %s\n", t.c_str());
  }
}

void procesarPaquete(Paquete &p) {
  p.texto[sizeof(p.texto) - 1] = '\0';

  if (p.origen == ID_CENTRAL) {
    centralPerdida = false;
    switch (p.tipo) {
      case AVISO:
        Serial.printf("[CENTRAL] %s\n", p.texto);
        break;
      case ENTRANTE:
        if (estado != REPOSO) break;
        pareja = p.dato;
        cambiarEstado(TIMBRANDO);
        Serial.printf("El nodo %u te llama\n", pareja);
        break;
      case CONECTAR:
        if (estado != LLAMANDO && estado != TIMBRANDO) break;
        pareja = p.dato;
        cambiarCanal(p.canal);
        cambiarEstado(EN_LLAMADA);
        Serial.printf("CONECTADO con nodo %u (canal %u)\n", pareja, p.canal);
        break;
      case RESPUESTA:
        if (p.dato != R_TIMBRANDO) { volverAReposo(false); }
        break;
    }
    return;
  }

  if (estado != EN_LLAMADA || p.origen != pareja) return;
  switch (p.tipo) {
    case TEXTO:
      Serial.printf("[MSG %u] %s\n", p.origen, p.texto);
      break;
    case COLGAR:
      volverAReposo(true);
      break;
  }
}

// Lee del puerto serie: lineas de texto (comandos) y tramas de audio 0xFE 0xFF + 29 bytes
void leerSerie() {
  static uint8_t fase = 0, pos = 0;
  static uint8_t cuerpo[CUERPO];
  static String linea = "";
  while (Serial.available()) {
    uint8_t c = Serial.read();
    if (fase == 2) {
      cuerpo[pos++] = c;
      if (pos == CUERPO) {
        fase = 0;
        if (estado == EN_LLAMADA) enviarVoz(cuerpo);
      }
      continue;
    }
    if (fase == 1) {
      fase = 0;
      if (c == 0xFF) { fase = 2; pos = 0; continue; }
    }
    if (c == 0xFE) fase = 1;
    else if (c == '\n' || c == '\r') { procesarLinea(linea); linea = ""; }
    else if (linea.length() < 100) linea += (char)c;
  }
}

void setup() {
  Serial.setRxBufferSize(2048);     // la PC manda las tramas de voz en rafagas
  Serial.begin(VELOCIDAD_SERIE);
  delay(1500);
  pinMode(PIN_LED, OUTPUT);
  ledBrillo(0);
  pinMode(PIN_BOOT, INPUT_PULLUP);

  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CSN);
  if (!radio.begin(&SPI)) {
    while (true) { ledBrillo(255); delay(100); ledBrillo(0); delay(100); }
  }
  radio.setChannel(CANAL_CONTROL);
  radio.setDataRate(RF24_1MBPS);
  radio.setPALevel(RF24_PA_LOW);
  radio.setPayloadSize(sizeof(Paquete));
  radio.setAutoAck(true);
  radio.enableDynamicAck();         // permite mandar la voz sin ACK
  radio.setRetries(3 + (MI_ID % 10), 15);
  uint8_t dir[5]; direccionDe(MI_ID, dir);
  radio.openReadingPipe(1, dir);
  radio.startListening();

  tRegistro = millis() - REGISTRO_CADA_MS;
  Serial.printf("Nodo telefono %u listo (%s)\n", MI_ID, PLACA);
}

void loop() {
  leerSerie();

  while (radio.available()) {
    uint8_t buf[32];
    radio.read(buf, sizeof(buf));
    if (buf[0] == VOZ) {
      if (estado == EN_LLAMADA && buf[1] == pareja) {
        nivelRemoto = constrain(buf[3] * 3, 0, 255);   // el indice ADPCM sirve de medidor de volumen
        if (audioPC) vozAlPC(buf + 3);
      }
    } else {
      Paquete p;
      memcpy(&p, buf, sizeof(p));
      procesarPaquete(p);
    }
  }

  // Registro periodico en la central (solo en reposo, cuando estamos en el canal de control)
  unsigned long ahora = millis();
  if (estado == REPOSO && ahora - tRegistro >= REGISTRO_CADA_MS) {
    tRegistro = ahora;
    bool ok = enviar(ID_CENTRAL, REGISTRO, 0, nullptr);
    if (ok) centralPerdida = false;
    else if (!centralPerdida) {
      centralPerdida = true;
      Serial.println("[ERR] La central no responde");
    }
  }

  // LED indicador
  uint8_t brillo = 0;
  if (estado == TIMBRANDO)  brillo = ((ahora / 150) % 2) ? 255 : 0;
  if (estado == LLAMANDO)   brillo = ((ahora / 600) % 2) ? 255 : 0;
  if (estado == EN_LLAMADA) brillo = max((uint8_t)6, gamma8(nivelRemoto));
  ledBrillo(brillo);
}
