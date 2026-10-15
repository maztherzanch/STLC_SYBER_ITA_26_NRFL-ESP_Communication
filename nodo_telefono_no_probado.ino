/*
  NODO TELEFONO v2 - Topologia estrella con central (nRF24L01+ PA+LNA + ESP32)
  Para la RED PROPIA con la central "central_telefonica_v2_2" y la interfaz HTML
  (Web Serial, "Audio ADPCM binario", 460800 baudios).

  La PC manda la voz ya comprimida (ADPCM 4 bits, 8 kHz) en tramas por el puerto serie:
      0xFE 0xFF + 29 bytes   (1 byte indice + 2 bytes predictor + 26 bytes de 52 muestras)
  El ESP32 solo la reenvia por radio al otro nodo y hace lo mismo en sentido contrario.
  Paquete de voz (32 bytes): [VOZ][origen][secuencia de voz][29 bytes de audio]

  Novedades respecto a la v1
   Red
    - NOMBRE_RED y CANAL_CONTROL propios (IGUALES que en la central v2.2).
   Llamadas mas estables
    - KEEPALIVE cada 2 s durante la llamada: si el otro nodo desaparece (se apago,
      se alejo) la llamada se cierra sola en ~8 s en vez de quedar colgada.
    - Los mensajes importantes (contestar, rechazar, colgar, liberar) se reintentan.
    - Tiempo maximo de espera para llamadas que nunca se conectan (30 s).
    - Entiende el COLGAR de la central (comando /liberar de la central).
    - Avisa el motivo cuando una llamada falla (ocupado, no disponible, etc.).
    - El registro en la central lleva un pequeno retraso aleatorio para que varios
      nodos no choquen siempre en el mismo instante.
   Audio
    - Secuencia de voz propia: mide cuantos paquetes de voz se pierden.
    - Cada 2 s manda a la PC una linea "CAL n" (n = % de paquetes perdidos).
    - Si se pierde un paquete, repite el anterior hacia la PC (oculta el hueco y
      mantiene el audio fluido en vez de dejar un silencio).
    - La lectura del puerto serie se resincroniza si se pierde un byte de una trama.
   Comandos nuevos: /info
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

#define VELOCIDAD_SERIE 460800   // debe coincidir con la velocidad elegida en el HTML

// ================= RED PROPIA (IDENTICO en la central y en TODOS los nodos) =================
#define NOMBRE_RED     "ITA1"    // exactamente 4 letras
#define CANAL_CONTROL  76
static_assert(sizeof(NOMBRE_RED) == 5, "NOMBRE_RED debe tener exactamente 4 letras");

#define ID_CENTRAL     0
#define CUERPO         29        // bytes de audio por paquete

// ------------------------------- Ajustes -------------------------------
#define REGISTRO_CADA_MS     5000UL
#define PENDIENTE_MAX_MS     30000UL   // LLAMANDO/TIMBRANDO sin que la central resuelva
#define KEEPALIVE_CADA_MS    2000UL
#define SIN_SENAL_MAX_MS     8000UL    // sin recibir nada del otro nodo = se perdio
#define INFORME_CADA_MS      2000UL    // linea "CAL n" hacia la PC
#define REINTENTOS           3
#define OCULTAR_PERDIDAS     1         // repetir el paquete anterior si se perdio uno

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
  dir[0] = id;
  dir[1] = NOMBRE_RED[0]; dir[2] = NOMBRE_RED[1]; dir[3] = NOMBRE_RED[2]; dir[4] = NOMBRE_RED[3];
}

RF24 radio(PIN_CE, PIN_CSN);
uint8_t secuencia = 0;           // paquetes de control
uint8_t secVoz = 0;              // paquetes de voz (aparte, para medir perdidas)

Estado estado = REPOSO;
uint8_t pareja = 0;
uint8_t canalActual = CANAL_CONTROL;
unsigned long tEstado = 0;
unsigned long tProxRegistro = 0;
bool centralPerdida = false;

// Durante la llamada
unsigned long tUltimoRx = 0, tKeepalive = 0, tInforme = 0;
uint8_t fallosKA = 0;
bool hayUltSec = false;
uint8_t ultSecRx = 0;
uint16_t rxVentana = 0, perdVentana = 0;
uint8_t ultimoCuerpo[CUERPO];
bool hayUltimoCuerpo = false;

bool audioPC = false;            // la PC pidio recibir audio (/audio 1)
uint8_t nivelRemoto = 0;         // solo para el brillo del LED

const char *nombreEstado[] = {"REPOSO", "LLAMANDO", "TIMBRANDO", "EN LLAMADA"};

// ------------------------------------------------------------------------------------
// ENVIO
Paquete armar(uint8_t destino, uint8_t tipo, uint8_t dato, const char *texto) {
  Paquete p = {};
  p.tipo = tipo; p.origen = MI_ID; p.destino = destino;
  p.canal = canalActual; p.dato = dato; p.secuencia = secuencia++;
  if (texto) strncpy(p.texto, texto, sizeof(p.texto) - 1);
  return p;
}

bool transmitir(const Paquete &p) {
  uint8_t dir[5]; direccionDe(p.destino, dir);
  radio.stopListening();
  radio.openWritingPipe(dir);
  bool ok = radio.write(&p, sizeof(p));
  radio.startListening();
  return ok;
}

bool enviar(uint8_t destino, uint8_t tipo, uint8_t dato, const char *texto = nullptr) {
  return transmitir(armar(destino, tipo, dato, texto));
}

// Para mensajes importantes: reintenta el MISMO paquete con una pausa aleatoria corta.
bool enviarSeguro(uint8_t destino, uint8_t tipo, uint8_t dato) {
  Paquete p = armar(destino, tipo, dato, nullptr);
  for (uint8_t i = 0; i < REINTENTOS; i++) {
    if (transmitir(p)) return true;
    delay(8 + random(16));
  }
  return false;
}

// Voz hacia el otro nodo: sin ACK, si se pierde un paquete no se reintenta
// (cada paquete lleva su propio estado ADPCM, asi que el decodificador se recupera solo).
void enviarVoz(const uint8_t *cuerpo) {
  uint8_t b[32];
  b[0] = VOZ; b[1] = MI_ID; b[2] = secVoz++;
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

// Voz recibida del otro nodo: mide perdidas, oculta huecos y la manda a la PC.
void recibirVoz(const uint8_t *b) {
  uint8_t seq = b[2], perdidos = 0;
  if (hayUltSec) {
    uint8_t salto = (uint8_t)(seq - ultSecRx);      // 1 = consecutivo
    if (salto == 0) return;                         // repetido
    if (salto > 1 && salto <= 64) perdidos = salto - 1;   // salto enorme = secuencia reiniciada, no cuenta
  }
  hayUltSec = true; ultSecRx = seq;
  rxVentana++; perdVentana += perdidos;
  nivelRemoto = constrain(b[3] * 3, 0, 255);        // el indice ADPCM sirve de medidor de volumen
  if (!audioPC) return;
#if OCULTAR_PERDIDAS
  if (hayUltimoCuerpo)
    for (uint8_t i = 0; i < perdidos && i < 3; i++) vozAlPC(ultimoCuerpo);
#endif
  vozAlPC(b + 3);
  memcpy(ultimoCuerpo, b + 3, CUERPO);
  hayUltimoCuerpo = true;
}

// ------------------------------------------------------------------------------------
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
  if (e == EN_LLAMADA) {                // contadores de la llamada nueva
    tUltimoRx = tKeepalive = tInforme = millis();
    fallosKA = 0;
    hayUltSec = false; rxVentana = perdVentana = 0;
    hayUltimoCuerpo = false;
  }
}

void ledBrillo(uint8_t v) {
  analogWrite(PIN_LED, LED_ON == HIGH ? v : 255 - v);
}

uint8_t gamma8(uint8_t v) { return (uint16_t)v * v / 255; }

void volverAReposo(bool avisarCentral) {
  if (canalActual != CANAL_CONTROL) cambiarCanal(CANAL_CONTROL);
  if (avisarCentral) enviarSeguro(ID_CENTRAL, LIBERAR, 0);
  pareja = 0;
  cambiarEstado(REPOSO);
  enviar(ID_CENTRAL, REGISTRO, 0);
  tProxRegistro = millis() + REGISTRO_CADA_MS + random(500);
  Serial.println("En reposo.");
}

const char *textoRespuesta(uint8_t c) {
  switch (c) {
    case R_OCUPADO:       return "El nodo esta ocupado";
    case R_NO_DISPONIBLE: return "El nodo no esta disponible";
    case R_RECHAZADO:     return "Llamada rechazada";
    case R_NO_CONTESTA:   return "El nodo no contesta";
    case R_SIN_CANALES:   return "No hay canales libres";
    case R_CANCELADO:     return "Llamada cancelada";
    default:              return "Respuesta desconocida de la central";
  }
}

// ------------------------------------------------------------------------------------
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
    if (destino < 1 || destino > 254 || destino == MI_ID) { Serial.println("[ERR] Numero de nodo invalido"); return; }
    if (!enviarSeguro(ID_CENTRAL, LLAMAR, destino)) {
      Serial.println("[ERR] La central no responde, no se pudo llamar");
      return;
    }
    pareja = destino;
    cambiarEstado(LLAMANDO);
    Serial.printf("Llamando al nodo %u\n", pareja);
  }
  else if (l == "/contestar") {
    if (estado != TIMBRANDO) return;
    if (!enviarSeguro(ID_CENTRAL, CONTESTAR, 0)) Serial.println("[ERR] La central no responde, intenta de nuevo");
  }
  else if (l == "/rechazar") {
    if (estado != TIMBRANDO) return;
    enviarSeguro(ID_CENTRAL, RECHAZAR, 0);
    volverAReposo(false);
  }
  else if (l == "/colgar") {
    if (estado == EN_LLAMADA) {
      enviarSeguro(pareja, COLGAR, 0);
      volverAReposo(true);
    } else if (estado == LLAMANDO || estado == TIMBRANDO) {
      volverAReposo(true);
    }
  }
  else if (l == "/estado") {
    if (pareja) Serial.printf("Nodo %u | %s | canal %u | con nodo %u\n", MI_ID, nombreEstado[estado], canalActual, pareja);
    else        Serial.printf("Nodo %u | %s | canal %u |\n", MI_ID, nombreEstado[estado], canalActual);
  }
  else if (l == "/info") {
    Serial.printf("Red \"%s\", canal de control %d, placa %s, audio a la PC: %s\n",
                  NOMBRE_RED, CANAL_CONTROL, PLACA, audioPC ? "si" : "no");
  }
  else if (l[0] == '/') {
    Serial.println("[ERR] Comando desconocido");
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
    if (centralPerdida) Serial.println("Central en linea de nuevo.");
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
        if (p.dato == R_TIMBRANDO || estado == REPOSO) break;
        Serial.printf("[LLAMADA] %s\n", textoRespuesta(p.dato));
        volverAReposo(false);
        break;
      case COLGAR:                      // la central termino la llamada (/liberar)
        if (estado != EN_LLAMADA) break;
        Serial.println("[CENTRAL] La central cerro la llamada");
        volverAReposo(false);
        break;
    }
    return;
  }

  if (estado != EN_LLAMADA || p.origen != pareja) return;
  tUltimoRx = millis();
  switch (p.tipo) {
    case TEXTO:
      Serial.printf("[MSG %u] %s\n", p.origen, p.texto);
      break;
    case COLGAR:
      volverAReposo(true);
      break;
    case KEEPALIVE:
      break;
  }
}

// Lee del puerto serie: lineas de texto (comandos) y tramas de audio 0xFE 0xFF + 29 bytes
void leerSerie() {
  static uint8_t fase = 0, pos = 0, prev = 0;
  static uint8_t cuerpo[CUERPO];
  static String linea = "";
  while (Serial.available()) {
    uint8_t c = Serial.read();
    if (fase == 2) {
      if (prev == 0xFE && c == 0xFF) { pos = 0; prev = 0; continue; }   // nueva cabecera: se perdio un byte, resincroniza
      prev = c;
      cuerpo[pos++] = c;
      if (pos == CUERPO) {
        fase = 0;
        if (estado == EN_LLAMADA) enviarVoz(cuerpo);
      }
      continue;
    }
    if (fase == 1) {
      fase = 0;
      if (c == 0xFF) { fase = 2; pos = 0; prev = 0; continue; }
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

  tProxRegistro = millis();         // registrarse enseguida
  Serial.printf("Nodo telefono %u listo (%s)\n", MI_ID, PLACA);
  Serial.printf("Red \"%s\", canal de control %d\n", NOMBRE_RED, CANAL_CONTROL);
}

void loop() {
  leerSerie();

  while (radio.available()) {
    uint8_t buf[32];
    radio.read(buf, sizeof(buf));
    if (buf[0] == VOZ) {
      if (estado == EN_LLAMADA && buf[1] == pareja) {
        tUltimoRx = millis();
        recibirVoz(buf);
      }
    } else {
      Paquete p;
      memcpy(&p, buf, sizeof(p));
      procesarPaquete(p);
    }
  }

  unsigned long ahora = millis();

  // Registro periodico en la central (solo en reposo, en el canal de control)
  if (estado == REPOSO && (long)(ahora - tProxRegistro) >= 0) {
    tProxRegistro = ahora + REGISTRO_CADA_MS + random(500);
    bool ok = enviar(ID_CENTRAL, REGISTRO, 0);
    if (ok) {
      if (centralPerdida) Serial.println("Central en linea de nuevo.");
      centralPerdida = false;
    } else if (!centralPerdida) {
      centralPerdida = true;
      Serial.println("[ERR] La central no responde");
    }
  }

  // Llamadas que nunca se resuelven
  if ((estado == LLAMANDO || estado == TIMBRANDO) && ahora - tEstado > PENDIENTE_MAX_MS) {
    Serial.println("[ERR] Sin respuesta de la central");
    volverAReposo(true);
  }

  // Durante la llamada: keepalive, vigilancia del otro nodo e informe de calidad
  if (estado == EN_LLAMADA) {
    if (ahora - tKeepalive >= KEEPALIVE_CADA_MS) {
      tKeepalive = ahora;
      if (enviar(pareja, KEEPALIVE, 0)) fallosKA = 0; else fallosKA++;
    }
    if (ahora - tUltimoRx > SIN_SENAL_MAX_MS || fallosKA >= 4) {
      Serial.println("Se perdio la conexion con el otro nodo.");
      volverAReposo(true);
    } else if (ahora - tInforme >= INFORME_CADA_MS) {
      tInforme = ahora;
      uint16_t total = rxVentana + perdVentana;
      if (total > 0 && audioPC) Serial.printf("CAL %u\n", (unsigned)(perdVentana * 100 / total));
      rxVentana = perdVentana = 0;
    }
  }

  // LED indicador
  uint8_t brillo = 0;
  if (estado == TIMBRANDO)  brillo = ((ahora / 150) % 2) ? 255 : 0;
  if (estado == LLAMANDO)   brillo = ((ahora / 600) % 2) ? 255 : 0;
  if (estado == EN_LLAMADA) brillo = max((uint8_t)6, gamma8(nivelRemoto));
  ledBrillo(brillo);
}
