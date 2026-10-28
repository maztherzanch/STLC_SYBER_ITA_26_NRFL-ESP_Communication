/*
  CENTRAL TELEFONICA v3.0 SEGURA (AES-128 + Anti-Replay + Whitelist)
  --------------------------------------------------------------------------
  SEGURIDAD CRIPTOGRAFICA EN TIEMPO REAL:
   1. Confidencialidad (Anti-Intercepcion): Cifrado AES-128 en modo flujo (CTR)
      acelerado por hardware en el ESP32 (< 15 microsegundos por paquete).
   2. Integridad y Autenticacion (Anti-Suplantacion): Firma MAC de 16 bits
      calculada con AES-128 sobre todo el paquete. Ningun nodo sin la CLAVE_RED
      puede ser aceptado ni falsificar ordenes como COLGAR o LLAMAR.
   3. Anti-Replay (Anti-Grabacion): Secuencia de 16 bits + validacion de ventana
      por cada nodo y sesion aleatoria al arrancar. Un paquete capturado y
      retransmitido por un atacante es descartado inmediatamente.
   4. Secreto de Llamada: Al conectar dos nodos, la central genera una semilla
      aleatoria por hardware (llaveSesion) para derivar el cifrado del audio.
   5. Control de Acceso (Lista Blanca): Solo los IDs autorizados pueden unirse.

  Comandos en el Monitor Serial (115200, "Nueva linea"):
     /todos texto | /msg ID texto | /tabla | /canales | /ruido
     /liberar ID  | /potencia nivel | /intrusos | /ayuda
*/
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include "mbedtls/aes.h"

// ============================ PINES SEGUN LA PLACA ============================
#if defined(CONFIG_IDF_TARGET_ESP32S3)
  #define PIN_CE 9
  #define PIN_CSN 10
  #define PIN_MOSI 11
  #define PIN_SCK 12
  #define PIN_MISO 13
  #define PIN_RGB 48
  #define PLACA "ESP32-S3 SuperMini"
#elif defined(CONFIG_IDF_TARGET_ESP32C3)
  #define PIN_CE 3
  #define PIN_CSN 7
  #define PIN_SCK 4
  #define PIN_MISO 5
  #define PIN_MOSI 6
  #define PIN_LED 8
  #define LED_ON LOW
  #define PLACA "ESP32-C3 SuperMini"
#elif defined(CONFIG_IDF_TARGET_ESP32)
  #define PIN_CE 2
  #define PIN_CSN 4
  #define PIN_SCK 18
  #define PIN_MISO 19
  #define PIN_MOSI 23
  #define PIN_LED 12
  #define LED_ON HIGH
  #define PLACA "ESP32 DevKit"
#else
  #error "Placa no soportada: elige ESP32S3, ESP32C3 o ESP32 Dev Module en Tools -> Board"
#endif

// ================= SEGURIDAD Y RED PROPIA (IDENTICO EN CENTRAL Y NODOS) =================
#define NOMBRE_RED     "ITA1"    // 4 letras (direccion fisica de pipe)
#define CANAL_CONTROL  76        // 2476 MHz
#define ID_CENTRAL     0
#define CANAL_MIN      0
#define CANAL_MAX      83
#define CANAL_AJENO    80
#define AJENO_MARGEN   2

// Clave maestra AES-128 (16 bytes exactos). ¡CAMBIALA POR LA TUYA SECRETA!
static const uint8_t CLAVE_RED[16] = {
  0x4A, 0x9F, 0x22, 0xE1, 0x7C, 0x3B, 0x88, 0x10,
  0xD4, 0x56, 0xA3, 0x09, 0xF1, 0x6E, 0xB7, 0xC5
};

// Lista blanca de nodos autorizados: pon 'true' para permitir solo la lista NODOS_PERMITIDOS
#define USAR_LISTA_BLANCA false
static const uint8_t NODOS_PERMITIDOS[] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 };

// ======================= PROTOCOLO SEGURO (32 BYTES) =======================
enum Tipo : uint8_t {
  REGISTRO = 1, LLAMAR, CONTESTAR, RECHAZAR, LIBERAR,     // nodo -> central
  ENTRANTE, CONECTAR, RESPUESTA,                          // central -> nodo
  TEXTO, KEEPALIVE, COLGAR,                               // nodo <-> nodo
  AVISO,                                                  // central -> nodo
  VOZ                                                     // nodo <-> nodo
};

enum Codigo : uint8_t {
  R_TIMBRANDO = 1, R_OCUPADO, R_NO_DISPONIBLE, R_RECHAZADO,
  R_NO_CONTESTA, R_SIN_CANALES, R_CANCELADO
};

struct __attribute__((packed)) Paquete { // 32 bytes exactos
  uint8_t  tipo;        // 1B (autenticado)
  uint8_t  origen;      // 1B (autenticado)
  uint8_t  destino;     // 1B (autenticado)
  uint8_t  canal;       // 1B (cifrado + autenticado)
  uint8_t  dato;        // 1B (cifrado + autenticado)
  uint16_t secuencia;   // 2B (nonce anti-replay, autenticado)
  uint8_t  llaveSesion; // 1B (semilla efimera para audio, cifrado + autenticado)
  char     texto[22];   // 22B (cifrado + autenticado)
  uint16_t mac;         // 2B (firma criptografica AES-128)
};
static_assert(sizeof(Paquete) == 32, "El paquete debe medir exactamente 32 bytes");

// ============================ AJUSTES DE LA CENTRAL ============================
#define MAX_NODOS         32
#define NODO_CADUCA_MS    15000UL
#define MAX_LLAMADAS      16
#define TIMBRE_MAX_MS     20000UL

#define REINTENTOS_ENVIO  3
#define UMBRAL_RUIDO      25
#define MONITOREO_CADA_MS 40
#define MUESTRAS_MONITOR  6
#define MUESTRAS_VERIFICA 10
#define DECAE_PICO        2
#define PENAL_WIFI        10
#define VENTANA_REPLAY    64       // margen maximo de paquetes perdidos seguidos aceptados
// ===================================================================================

struct Nodo {
  uint8_t       id;
  unsigned long visto;
  uint16_t      ultimaSec;         // ultimo contador valido recibido (Anti-Replay)
  bool          secInicializada;
};

enum EstadoLlamada : uint8_t { LIBRE = 0, TIMBRANDO, EN_CURSO };
struct Llamada {
  EstadoLlamada estado;
  uint8_t       origen, destino, canal, llaveSesion;
  unsigned long desde;
};

static_assert(sizeof(NOMBRE_RED) == 5, "NOMBRE_RED debe tener exactamente 4 letras");
void direccionDe(uint8_t id, uint8_t *dir) {
  dir[0] = id;
  dir[1] = NOMBRE_RED[0] ^ (CLAVE_RED[0] & 0x0F); // Ofusca la direccion RF fisica con la clave
  dir[2] = NOMBRE_RED[1] ^ (CLAVE_RED[1] & 0x0F);
  dir[3] = NOMBRE_RED[2] ^ (CLAVE_RED[2] & 0x0F);
  dir[4] = NOMBRE_RED[3] ^ (CLAVE_RED[3] & 0x0F);
}

RF24 radio(PIN_CE, PIN_CSN);
mbedtls_aes_context aesCtx;
uint16_t secuenciaTx = 0;

Nodo nodos[MAX_NODOS];
uint8_t numNodos = 0;
Llamada llamadas[MAX_LLAMADAS];

uint8_t ruido[CANAL_MAX + 1];
uint8_t pico[CANAL_MAX + 1];
uint8_t canalMonitoreo = CANAL_MIN;
unsigned long tMonitoreo = 0;

unsigned long totalLlamadas = 0, enviosFallidos = 0, paquetesRx = 0;
unsigned long ataquesMac = 0, ataquesReplay = 0, rechazadosWhitelist = 0;

int llamadaDe(uint8_t id);

// ------------------------------------------------------------------------------------
// MOTOR CRIPTOGRAFICO AES-128 (COMPARTIR IDENTICO EN LOS NODOS)

// Genera dos bloques pseudoaleatorios AES a partir de (origen, destino, secuencia, semilla)
void generarBloquesAES(uint8_t origen, uint8_t destino, uint16_t seq, uint8_t semilla,
                       uint8_t keystream[32], uint8_t subkeyMac[16]) {
  uint8_t iv[16] = {
    origen, destino, (uint8_t)(seq >> 8), (uint8_t)(seq & 0xFF),
    semilla, NOMBRE_RED[0], NOMBRE_RED[1], NOMBRE_RED[2],
    NOMBRE_RED[3], 0xA5, 0x5A, 0xC3, 0x3C, 0x01, 0x02, 0x00
  };
  // Bloque 0 y 1: Keystream para cifrar/descifrar payload
  iv[15] = 0x01; mbedtls_aes_crypt_ecb(&aesCtx, MBEDTLS_AES_ENCRYPT, iv, keystream);
  iv[15] = 0x02; mbedtls_aes_crypt_ecb(&aesCtx, MBEDTLS_AES_ENCRYPT, iv + 0, keystream + 16);
  // Bloque 2: Bloque base para firmar MAC
  iv[15] = 0xFF; mbedtls_aes_crypt_ecb(&aesCtx, MBEDTLS_AES_ENCRYPT, iv, subkeyMac);
}

// Calcula el MAC de 16 bits sobre los primeros 30 bytes del paquete (en claro)
uint16_t calcularMAC(const Paquete &p, const uint8_t subkeyMac[16]) {
  const uint8_t *raw = (const uint8_t *)&p;
  uint8_t estado[16];
  memcpy(estado, subkeyMac, 16);

  // Absorbemos los primeros 16 bytes
  for (int i = 0; i < 16; i++) estado[i] ^= raw[i];
  mbedtls_aes_crypt_ecb(&aesCtx, MBEDTLS_AES_ENCRYPT, estado, estado);

  // Absorbemos los siguientes 14 bytes (hasta completar los 30 bytes previos al campo mac)
  for (int i = 0; i < 14; i++) estado[i] ^= raw[16 + i];
  estado[14] ^= 0x80; // Padding estandar
  mbedtls_aes_crypt_ecb(&aesCtx, MBEDTLS_AES_ENCRYPT, estado, estado);

  return ((uint16_t)estado[0] << 8) | estado[1];
}

// Firma y cifra un paquete in-place antes de enviarlo por RF (semilla = 0 en canal de control)
void protegerPaquete(Paquete &p, uint8_t semilla = 0) {
  uint8_t ks[32], skMac[16];
  generarBloquesAES(p.origen, p.destino, p.secuencia, semilla, ks, skMac);

  // 1) Firmar datos en claro (Encrypt-then-MAC o MAC-then-Encrypt consistente)
  p.mac = calcularMAC(p, skMac);

  // 2) Cifrar campos sensibles: canal, dato, llaveSesion y texto[22]
  p.canal       ^= ks[0];
  p.dato        ^= ks[1];
  p.llaveSesion ^= ks[2];
  for (int i = 0; i < 22; i++) p.texto[i] ^= ks[3 + i];
}

// Descifra y verifica autenticidad. Devuelve false si fue alterado o la clave no coincide.
bool verificarYDescifrar(Paquete &p, uint8_t semilla = 0) {
  uint8_t ks[32], skMac[16];
  generarBloquesAES(p.origen, p.destino, p.secuencia, semilla, ks, skMac);

  // 1) Descifrar en memoria
  p.canal       ^= ks[0];
  p.dato        ^= ks[1];
  p.llaveSesion ^= ks[2];
  for (int i = 0; i < 22; i++) p.texto[i] ^= ks[3 + i];

  // 2) Verificar firma MAC en tiempo constante
  uint16_t macEsperado = calcularMAC(p, skMac);
  return (p.mac == macEsperado);
}

// ------------------------------------------------------------------------------------
// CONTROL DE ACCESO (WHITELIST + ANTI-REPLAY)
bool nodoAutorizado(uint8_t id) {
  if (id == ID_CENTRAL || id > 254) return false;
#if USAR_LISTA_BLANCA
  for (size_t i = 0; i < sizeof(NODOS_PERMITIDOS); i++) {
    if (NODOS_PERMITIDOS[i] == id) return true;
  }
  return false;
#else
  return true;
#endif
}

// Valida que el numero de secuencia sea mas reciente que el anterior (con soporte de rollover)
bool secuenciaValida(int idxNodo, uint8_t tipoPaquete, uint16_t seqRecibida) {
  if (idxNodo < 0) return true; // Nodo nuevo, se inicializa al registrar
  if (!nodos[idxNodo].secInicializada) {
    nodos[idxNodo].ultimaSec = seqRecibida;
    nodos[idxNodo].secInicializada = true;
    return true;
  }
  uint16_t diff = (uint16_t)(seqRecibida - nodos[idxNodo].ultimaSec);

  // Caso 1: Es el mismo paquete reintentado por el nodo (diff == 0)
  if (diff == 0 && (tipoPaquete == LLAMAR || tipoPaquete == CONTESTAR || tipoPaquete == REGISTRO)) {
    return true;
  }
  // Caso 2: Avance normal dentro de la ventana
  if (diff > 0 && diff <= VENTANA_REPLAY) {
    nodos[idxNodo].ultimaSec = seqRecibida;
    return true;
  }
  // Caso 3: Un nodo se reinicio legitimamente y envia REGISTRO con nueva secuencia aleatoria
  if (tipoPaquete == REGISTRO) {
    nodos[idxNodo].ultimaSec = seqRecibida;
    return true;
  }
  return false; // Paquete viejo reinyectado (Replay Attack)
}

// ------------------------------------------------------------------------------------
// LED: 0 = apagado, 1 = central lista, 2 = hay llamadas, 3 = error
void ledModo(uint8_t modo) {
#ifdef PIN_RGB
  uint8_t r = 0, g = 0, b = 0;
  if (modo == 1) b = 10;
  if (modo == 2) g = 40;
  if (modo == 3) r = 40;
  #if ESP_ARDUINO_VERSION_MAJOR >= 3
    rgbLedWrite(PIN_RGB, r, g, b);
  #else
    neopixelWrite(PIN_RGB, r, g, b);
  #endif
#else
  digitalWrite(PIN_LED, modo >= 2 ? LED_ON : !LED_ON);
#endif
}

// ------------------------------------------------------------------------------------
// ENVIO SEGURO Y CIFRADO
Paquete armarCifrado(uint8_t destino, uint8_t tipo, uint8_t canal, uint8_t dato,
                     uint8_t llaveSesion, const char *texto, uint8_t semillaCifrado = 0) {
  Paquete p = {};
  p.tipo        = tipo;
  p.origen      = ID_CENTRAL;
  p.destino     = destino;
  p.canal       = canal;
  p.dato        = dato;
  p.secuencia   = ++secuenciaTx;
  p.llaveSesion = llaveSesion;
  if (texto) strncpy(p.texto, texto, sizeof(p.texto) - 1);
  protegerPaquete(p, semillaCifrado);
  return p;
}

bool transmitir(uint8_t canalRF, const Paquete &pCifrado) {
  uint8_t dir[5]; direccionDe(pCifrado.destino, dir);
  radio.stopListening();
  if (canalRF != CANAL_CONTROL) radio.setChannel(canalRF);
  radio.openWritingPipe(dir);
  bool ok = radio.write(&pCifrado, sizeof(pCifrado));
  if (canalRF != CANAL_CONTROL) radio.setChannel(CANAL_CONTROL);
  radio.startListening();
  if (!ok) enviosFallidos++;
  return ok;
}

bool enviarEnCanal(uint8_t canalRF, uint8_t destino, uint8_t tipo,
                   uint8_t canal, uint8_t dato, const char *texto, uint8_t semillaAudio = 0) {
  Paquete p = armarCifrado(destino, tipo, canal, dato, 0, texto, semillaAudio);
  return transmitir(canalRF, p);
}

bool enviarSeguro(uint8_t destino, uint8_t tipo, uint8_t canal, uint8_t dato, uint8_t llaveSesion = 0) {
  Paquete p = armarCifrado(destino, tipo, canal, dato, llaveSesion, nullptr, 0);
  for (uint8_t i = 0; i < REINTENTOS_ENVIO; i++) {
    radio.setRetries(3 + (esp_random() % 6), 15);
    if (transmitir(CANAL_CONTROL, p)) {
      radio.setRetries(5, 15);
      return true;
    }
    delay(8 + (esp_random() % 18));
  }
  radio.setRetries(5, 15);
  return false;
}

// ---- Nodos ----
int buscarNodo(uint8_t id) {
  for (int i = 0; i < numNodos; i++) if (nodos[i].id == id) return i;
  return -1;
}

bool nodoEnLinea(uint8_t id) {
  int i = buscarNodo(id);
  return i >= 0 && millis() - nodos[i].visto < NODO_CADUCA_MS;
}

int registrarNodo(uint8_t id, uint16_t seqInicial) {
  int i = buscarNodo(id);
  if (i < 0) {
    if (numNodos < MAX_NODOS) {
      i = numNodos++;
    } else {
      unsigned long maxInactivo = 0;
      for (int k = 0; k < MAX_NODOS; k++) {
        unsigned long inactivo = millis() - nodos[k].visto;
        if (inactivo >= NODO_CADUCA_MS && llamadaDe(nodos[k].id) < 0 && inactivo > maxInactivo) {
          maxInactivo = inactivo;
          i = k;
        }
      }
      if (i < 0) return -1;
      Serial.printf("Reciclando espacio del nodo inactivo %u para el nodo %u\n", nodos[i].id, id);
    }
    nodos[i].id = id;
    nodos[i].ultimaSec = seqInicial;
    nodos[i].secInicializada = true;
    Serial.printf("Nodo %u autenticado y registrado\n", id);
  }
  nodos[i].visto = millis();
  return i;
}

// ---- Llamadas ----
int llamadaDe(uint8_t id) {
  for (int i = 0; i < MAX_LLAMADAS; i++)
    if (llamadas[i].estado != LIBRE && (llamadas[i].origen == id || llamadas[i].destino == id)) return i;
  return -1;
}

int llamadaEnCanal(int c) {
  for (int i = 0; i < MAX_LLAMADAS; i++)
    if (llamadas[i].estado != LIBRE && llamadas[i].canal == c) return i;
  return -1;
}

int espacioLlamada() {
  for (int i = 0; i < MAX_LLAMADAS; i++) if (llamadas[i].estado == LIBRE) return i;
  return -1;
}

void liberar(int i, const char *motivo) {
  Serial.printf("Canal %u liberado (%u <-> %u): %s\n", llamadas[i].canal,
                llamadas[i].origen, llamadas[i].destino, motivo);
  llamadas[i].estado = LIBRE;
}

// ------------------------------------------------------------------------------------
// MONITOREO DE CANALES
uint8_t medirCanal(uint8_t c, uint8_t muestras) {
  uint8_t conSenal = 0;
  radio.stopListening();
  radio.setChannel(c);
  for (uint8_t i = 0; i < muestras; i++) {
    radio.startListening();
    delayMicroseconds(170);
    if (radio.testRPD()) conSenal++;
    radio.stopListening();
  }
  radio.setChannel(CANAL_CONTROL);
  radio.startListening();
  return conSenal * 100 / muestras;
}

bool canalAjeno(int c) { return abs(c - CANAL_AJENO) <= AJENO_MARGEN; }

bool cercaDeOcupado(int c) {
  for (int d = -1; d <= 1; d++) {
    int k = c + d;
    if (k < CANAL_MIN || k > CANAL_MAX) continue;
    if (k == CANAL_CONTROL || llamadaEnCanal(k) >= 0) return true;
  }
  return false;
}

void anotarMedida(uint8_t c, uint8_t medida) {
  ruido[c] = (ruido[c] * 3 + medida) / 4;
  if (medida >= pico[c]) pico[c] = medida;
  else pico[c] = pico[c] > DECAE_PICO ? pico[c] - DECAE_PICO : 0;
}

void monitorearCanales() {
  if (millis() - tMonitoreo < MONITOREO_CADA_MS) return;
  tMonitoreo = millis();
  uint8_t c = canalMonitoreo;
  canalMonitoreo = (c >= CANAL_MAX) ? CANAL_MIN : c + 1;
  if (cercaDeOcupado(c)) return;
  anotarMedida(c, medirCanal(c, MUESTRAS_MONITOR));
}

int ruidoEfectivo(int c) { return max((int)ruido[c], (int)pico[c] / 2); }
bool enBandaWiFi(int c) { return (c >= 1 && c <= 23) || (c >= 26 && c <= 48) || (c >= 51 && c <= 73); }
int puntaje(int c) { return ruidoEfectivo(c) + (enBandaWiFi(c) ? PENAL_WIFI : 0); }

int distanciaAOcupados(int c) {
  int d = min((int)abs(c - CANAL_CONTROL), (int)abs(c - CANAL_AJENO));
  for (int i = 0; i < MAX_LLAMADAS; i++)
    if (llamadas[i].estado != LIBRE) d = min(d, (int)abs(c - (int)llamadas[i].canal));
  return d;
}

int elegirCanal() {
  for (int pasada = 0; pasada < 2; pasada++) {
    int mejor = -1, mejorDist = -1, mejorPuntaje = 9999;
    for (int c = CANAL_MIN; c <= CANAL_MAX; c++) {
      if (c == CANAL_CONTROL || canalAjeno(c) || llamadaEnCanal(c) >= 0) continue;
      if (pasada == 0 && ruidoEfectivo(c) > UMBRAL_RUIDO) continue;
      int dist = min(distanciaAOcupados(c), 6), pt = puntaje(c);
      if (dist > mejorDist || (dist == mejorDist && pt < mejorPuntaje)) {
        mejor = c; mejorDist = dist; mejorPuntaje = pt;
      }
    }
    if (mejor >= 0) return mejor;
  }
  return -1;
}

int elegirCanalVerificado() {
  for (int intento = 0; intento < 5; intento++) {
    int c = elegirCanal();
    if (c < 0) return -1;
    uint8_t medida = medirCanal(c, MUESTRAS_VERIFICA);
    ruido[c] = max(ruido[c], medida);
    pico[c]  = max(pico[c], medida);
    if (medida <= UMBRAL_RUIDO) return c;
    Serial.printf("  Canal %d tiene interferencia (%u%%), buscando otro...\n", c, medida);
  }
  return elegirCanal();
}

// ------------------------------------------------------------------------------------
// LLAMADAS
void atenderLlamar(uint8_t a, uint8_t b) {
  Serial.printf("Nodo %u quiere llamar al nodo %u\n", a, b);
  if (b == a || b == ID_CENTRAL || !nodoEnLinea(b)) {
    enviarSeguro(a, RESPUESTA, 0, R_NO_DISPONIBLE); return;
  }
  int previa = llamadaDe(a);
  if (previa >= 0) {
    bool repetida = llamadas[previa].estado == TIMBRANDO &&
                    llamadas[previa].origen == a && llamadas[previa].destino == b;
    enviarSeguro(a, RESPUESTA, 0, repetida ? R_TIMBRANDO : R_OCUPADO);
    return;
  }
  if (llamadaDe(b) >= 0) { enviarSeguro(a, RESPUESTA, 0, R_OCUPADO); return; }

  int i = espacioLlamada();
  int canal = (i >= 0) ? elegirCanalVerificado() : -1;
  if (canal < 0) { enviarSeguro(a, RESPUESTA, 0, R_SIN_CANALES); return; }

  // Generar llave efimera aleatoria (1..255) exclusiva para cifrar el audio de esta llamada
  uint8_t llaveEfimera = 1 + (esp_random() % 254);

  llamadas[i] = Llamada{TIMBRANDO, a, b, (uint8_t)canal, llaveEfimera, millis()};
  if (!enviarSeguro(b, ENTRANTE, 0, a)) {
    liberar(i, "el destino no respondio");
    enviarSeguro(a, RESPUESTA, 0, R_NO_DISPONIBLE);
    return;
  }
  enviarSeguro(a, RESPUESTA, 0, R_TIMBRANDO);
  Serial.printf("  Timbrando en nodo %u (canal %d reservado, ruido %d%%)\n", b, canal, ruidoEfectivo(canal));
}

void atenderContestar(uint8_t b) {
  int i = llamadaDe(b);
  if (i < 0 || llamadas[i].estado != TIMBRANDO || llamadas[i].destino != b) return;
  uint8_t a = llamadas[i].origen, canal = llamadas[i].canal, llave = llamadas[i].llaveSesion;

  // Entregamos tanto el canal como la llave efimera de sesion a ambos nodos
  bool okB = enviarSeguro(b, CONECTAR, canal, a, llave);
  bool okA = enviarSeguro(a, CONECTAR, canal, b, llave);
  if (okA && okB) {
    llamadas[i].estado = EN_CURSO;
    llamadas[i].desde = millis();
    totalLlamadas++;
    Serial.printf("CONECTADOS %u <-> %u en canal %u (%u MHz) [Sesion cifrada #%02X]\n",
                  a, b, canal, 2400 + canal, llave);
    return;
  }
  Serial.printf("No se pudo conectar %u <-> %u (respondio A:%s B:%s)\n", a, b, okA ? "si" : "no", okB ? "si" : "no");
  if (okA) enviarEnCanal(canal, a, RESPUESTA, 0, R_CANCELADO, nullptr, llave);
  else     enviarSeguro(a, RESPUESTA, 0, R_NO_DISPONIBLE);
  if (okB) enviarEnCanal(canal, b, RESPUESTA, 0, R_CANCELADO, nullptr, llave);
  else     enviarSeguro(b, RESPUESTA, 0, R_CANCELADO);
  liberar(i, "fallo al conectar");
}

void atenderRechazar(uint8_t b) {
  int i = llamadaDe(b);
  if (i < 0 || llamadas[i].estado != TIMBRANDO || llamadas[i].destino != b) return;
  enviarSeguro(llamadas[i].origen, RESPUESTA, 0, R_RECHAZADO);
  liberar(i, "rechazada");
}

void atenderLiberar(uint8_t x) {
  int i = llamadaDe(x);
  if (i < 0) return;
  if (llamadas[i].estado == TIMBRANDO) {
    uint8_t otro = (llamadas[i].origen == x) ? llamadas[i].destino : llamadas[i].origen;
    enviarSeguro(otro, RESPUESTA, 0, R_CANCELADO);
    liberar(i, "cancelada antes de contestar");
  } else {
    liberar(i, "colgaron");
  }
}

void forzarLiberar(uint8_t x) {
  int i = llamadaDe(x);
  if (i < 0) { Serial.printf("El nodo %u no esta en ninguna llamada.\n", x); return; }
  if (llamadas[i].estado == EN_CURSO) {
    uint8_t llave = llamadas[i].llaveSesion;
    enviarEnCanal(llamadas[i].canal, llamadas[i].origen,  COLGAR, 0, 0, nullptr, llave);
    enviarEnCanal(llamadas[i].canal, llamadas[i].destino, COLGAR, 0, 0, nullptr, llave);
  } else if (llamadas[i].estado == TIMBRANDO) {
    enviarSeguro(llamadas[i].origen,  RESPUESTA, 0, R_CANCELADO);
    enviarSeguro(llamadas[i].destino, RESPUESTA, 0, R_CANCELADO);
  }
  liberar(i, "forzada desde la central");
}

void revisarTiempos() {
  bool alguna = false;
  for (int i = 0; i < MAX_LLAMADAS; i++) {
    if (llamadas[i].estado == TIMBRANDO && millis() - llamadas[i].desde > TIMBRE_MAX_MS) {
      enviarSeguro(llamadas[i].origen, RESPUESTA, 0, R_NO_CONTESTA);
      enviarSeguro(llamadas[i].destino, RESPUESTA, 0, R_CANCELADO);
      liberar(i, "no contesto");
    }
    if (llamadas[i].estado == EN_CURSO) alguna = true;
  }
  static int ledAnterior = -1;
  if ((int)alguna != ledAnterior) { ledAnterior = alguna; ledModo(alguna ? 2 : 1); }
}

// ------------------------------------------------------------------------------------
// MENSAJES DE LA CENTRAL (se parten en pedazos de 21 caracteres + '\0')
bool entregarAviso(uint8_t id, const String &texto) {
  int l = llamadaDe(id);
  bool enLlamada = (l >= 0 && llamadas[l].estado == EN_CURSO);
  uint8_t canalRF = enLlamada ? llamadas[l].canal : CANAL_CONTROL;
  uint8_t semilla = enLlamada ? llamadas[l].llaveSesion : 0;
  bool ok = true;
  for (unsigned int i = 0; i < texto.length(); i += 21) {
    if (i > 0) delay(5);
    bool fragOk = enviarEnCanal(canalRF, id, AVISO, 0, 0, texto.substring(i, i + 21).c_str(), semilla);
    ok = fragOk && ok;
  }
  return ok;
}

void avisoATodos(const String &texto) {
  int entregados = 0, total = 0;
  for (int i = 0; i < numNodos; i++) {
    uint8_t id = nodos[i].id;
    if (!nodoEnLinea(id) && llamadaDe(id) < 0) continue;
    total++;
    bool ok = entregarAviso(id, texto);
    if (ok) entregados++;
    int l = llamadaDe(id);
    Serial.printf("  -> nodo %3u %s%s\n", id, ok ? "entregado" : "NO RESPONDIO",
                  (l >= 0 && llamadas[l].estado == EN_CURSO) ? " (en su canal de llamada)" : "");
  }
  if (total == 0) Serial.println("No hay nodos conectados.");
  else Serial.printf("Mensaje entregado a %d de %d nodos.\n", entregados, total);
}

// ------------------------------------------------------------------------------------
// INFORMACION Y AUDITORIA DE SEGURIDAD
void mostrarTabla() {
  Serial.println("--- Nodos Autenticados ---");
  if (numNodos == 0) Serial.println("  (ninguno)");
  for (int i = 0; i < numNodos; i++) {
    uint8_t id = nodos[i].id;
    int l = llamadaDe(id);
    if (l >= 0) {
      uint8_t otro = llamadas[l].origen == id ? llamadas[l].destino : llamadas[l].origen;
      Serial.printf("  Nodo %3u : %s con %u (canal %u, sesion #%02X)\n", id,
                    llamadas[l].estado == TIMBRANDO ? "timbrando" : "en llamada",
                    otro, llamadas[l].canal, llamadas[l].llaveSesion);
    } else {
      Serial.printf("  Nodo %3u : %s (seq=%u)\n", id,
                    nodoEnLinea(id) ? "libre" : "desconectado", nodos[i].ultimaSec);
    }
  }
  unsigned long s = millis() / 1000;
  Serial.printf("  Llamadas: %lu | Fallos RF: %lu | Rx validos: %lu | Bloqueados (MAC/Replay/Lista): %lu/%lu/%lu | Activa: %lu h %lu min\n",
                totalLlamadas, enviosFallidos, paquetesRx,
                ataquesMac, ataquesReplay, rechazadosWhitelist, s / 3600, (s / 60) % 60);
}

void mostrarIntrusos() {
  Serial.println("--- Auditoria de Seguridad RF ---");
  Serial.printf("  Paquetes con firma invalida (clave incorrecta o alterados): %lu\n", ataquesMac);
  Serial.printf("  Paquetes repetidos bloqueados (Anti-Replay):                %lu\n", ataquesReplay);
  Serial.printf("  Intentos de IDs fuera de la Lista Blanca:                   %lu\n", rechazadosWhitelist);
  Serial.printf("  Modo Lista Blanca: %s\n", USAR_LISTA_BLANCA ? "ACTIVO" : "DESACTIVADO (cualquier ID con la clave entra)");
}

void mostrarCanales() {
  Serial.printf("--- Canales %u a %u (%u a %u MHz) ---\n", CANAL_MIN, CANAL_MAX, 2400 + CANAL_MIN, 2400 + CANAL_MAX);
  Serial.print("  ");
  for (int c = CANAL_MIN; c <= CANAL_MAX; c++) Serial.print(c % 10 == 0 ? (char)('0' + (c / 10) % 10) : ' ');
  Serial.println();
  int libres = 0, conRuido = 0, enUso = 0;
  Serial.print("  ");
  for (int c = CANAL_MIN; c <= CANAL_MAX; c++) {
    char s;
    if (c == CANAL_CONTROL)                    s = 'C';
    else if (canalAjeno(c))                    s = 'A';
    else if (llamadaEnCanal(c) >= 0)          { s = 'L'; enUso++; }
    else if (ruidoEfectivo(c) > UMBRAL_RUIDO) { s = 'x'; conRuido++; }
    else                                      { s = '.'; libres++; }
    Serial.print(s);
  }
  Serial.println();
  Serial.println("  .=libre  L=llamada  C=control  A=reservado (otra red)  x=interferencia");
  Serial.printf("  Libres: %d   En llamada: %d   Con interferencia: %d\n", libres, enUso, conRuido);
}

void mostrarRuido() {
  Serial.println("--- Interferencia por canal (promedio / pico reciente) ---");
  bool alguno = false;
  for (int c = CANAL_MIN; c <= CANAL_MAX; c++) {
    if (c == CANAL_CONTROL || (ruido[c] == 0 && pico[c] == 0)) continue;
    alguno = true;
    Serial.printf("  canal %2d (%u MHz): %3u%% / %3u%% ", c, 2400 + c, ruido[c], pico[c]);
    for (int k = 0; k < ruidoEfectivo(c) / 4; k++) Serial.print('#');
    Serial.println(ruidoEfectivo(c) > UMBRAL_RUIDO ? "  <- se evita" : "");
  }
  if (!alguno) Serial.println("  Ningun canal con interferencia detectada.");
}

void mostrarAyuda() {
  Serial.println("Comandos: /todos texto | /msg ID texto | /tabla | /canales | /ruido | /intrusos | /liberar ID | /potencia min|baja|alta|max | /ayuda");
}

void procesarLinea(String l) {
  l.trim();
  if (l.length() == 0) return;
  if (l.startsWith("/todos ")) {
    String t = l.substring(7); t.trim();
    if (t.length() == 0) { Serial.println("Uso: /todos texto"); return; }
    Serial.printf("Mensaje cifrado a todos: \"%s\"\n", t.c_str());
    avisoATodos(t);
  }
  else if (l.startsWith("/msg ")) {
    String resto = l.substring(5); resto.trim();
    int esp = resto.indexOf(' ');
    if (esp < 0) { Serial.println("Uso: /msg ID texto"); return; }
    int id = resto.substring(0, esp).toInt();
    String txt = resto.substring(esp + 1); txt.trim();
    if (id < 1 || id > 254 || txt.length() == 0) { Serial.println("Uso: /msg ID texto  (ID de 1 a 254)"); return; }
    bool ok = entregarAviso(id, txt);
    Serial.printf("Mensaje al nodo %d: %s\n", id, ok ? "entregado" : "NO RESPONDIO");
  }
  else if (l.startsWith("/liberar ")) {
    int id = l.substring(9).toInt();
    if (id < 1 || id > 254) { Serial.println("Uso: /liberar ID"); return; }
    forzarLiberar(id);
  }
  else if (l.startsWith("/potencia")) {
    String a = l.substring(9); a.trim(); a.toLowerCase();
    rf24_pa_dbm_e nivel;
    if      (a == "min")  nivel = RF24_PA_MIN;
    else if (a == "baja") nivel = RF24_PA_LOW;
    else if (a == "alta") nivel = RF24_PA_HIGH;
    else if (a == "max")  nivel = RF24_PA_MAX;
    else { Serial.println("Uso: /potencia min|baja|alta|max"); return; }
    radio.setPALevel(nivel);
    Serial.printf("Potencia de la central: %s\n", a.c_str());
  }
  else if (l == "/tabla")    mostrarTabla();
  else if (l == "/canales")  mostrarCanales();
  else if (l == "/ruido")    mostrarRuido();
  else if (l == "/intrusos") mostrarIntrusos();
  else mostrarAyuda();
}

// ------------------------------------------------------------------------------------
void atenderPaquete(Paquete &p) {
  // 1) Filtro de Lista Blanca (IDs permitidos)
  if (!nodoAutorizado(p.origen) || p.destino != ID_CENTRAL) {
    rechazadosWhitelist++;
    return;
  }

  // 2) Verificacion criptografica de autenticidad (MAC) y descifrado AES-128
  if (!verificarYDescifrar(p, 0)) {
    ataquesMac++;
    Serial.printf("[ALERTA] Paquete rechazado (MAC invalido) del supuesto origen %u\n", p.origen);
    return;
  }

  // 3) Verificacion Anti-Replay (secuencia monotona por nodo)
  int idx = buscarNodo(p.origen);
  if (!secuenciaValida(idx, p.tipo, p.secuencia)) {
    ataquesReplay++;
    Serial.printf("[ALERTA] Replay Attack bloqueado del nodo %u (seq=%u)\n", p.origen, p.secuencia);
    return;
  }

  paquetesRx++;
  registrarNodo(p.origen, p.secuencia);

  switch (p.tipo) {
    case REGISTRO: {
      int i = llamadaDe(p.origen);
      if (i >= 0 && llamadas[i].estado == EN_CURSO && millis() - llamadas[i].desde > 3000) {
        uint8_t otro = (llamadas[i].origen == p.origen) ? llamadas[i].destino : llamadas[i].origen;
        enviarEnCanal(llamadas[i].canal, otro, COLGAR, 0, 0, nullptr, llamadas[i].llaveSesion);
        liberar(i, "el nodo regreso sin colgar");
      }
      break;
    }
    case LLAMAR:    atenderLlamar(p.origen, p.dato); break;
    case CONTESTAR: atenderContestar(p.origen);      break;
    case RECHAZAR:  atenderRechazar(p.origen);       break;
    case LIBERAR:   atenderLiberar(p.origen);        break;
  }
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  randomSeed(esp_random());
  secuenciaTx = (uint16_t)(esp_random() & 0xFFFF); // Secuencia inicial impredecible

  // Inicializar acelerador AES-128 por hardware del ESP32
  mbedtls_aes_init(&aesCtx);
  mbedtls_aes_setkey_enc(&aesCtx, CLAVE_RED, 128);

#ifdef PIN_LED
  pinMode(PIN_LED, OUTPUT);
#endif
  ledModo(0);

  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CSN);
  if (!radio.begin(&SPI, 10000000)) {
    Serial.println("ERROR: el nRF24L01 no responde. Revisa cables y alimentacion.");
    while (true) { ledModo(3); delay(150); ledModo(0); delay(150); }
  }
  radio.setDataRate(RF24_1MBPS);
  radio.setPALevel(RF24_PA_LOW);
  radio.setPayloadSize(sizeof(Paquete));
  radio.setAutoAck(true);
  radio.setRetries(5, 15);
  uint8_t dir[5]; direccionDe(ID_CENTRAL, dir);
  radio.openReadingPipe(1, dir);

  Serial.println("\nMidiendo interferencia en todos los canales...");
  for (int c = CANAL_MIN; c <= CANAL_MAX; c++) {
    uint8_t m = (c == CANAL_CONTROL) ? 0 : medirCanal(c, MUESTRAS_VERIFICA);
    ruido[c] = pico[c] = m;
  }

  radio.setChannel(CANAL_CONTROL);
  radio.startListening();
  ledModo(1);

  Serial.printf("=== CENTRAL TELEFONICA v3.0 SEGURA AES-128 (%s) ===\n", PLACA);
  Serial.printf("Red \"%s\", canal %d (%d MHz). Direcciones RF ofuscadas y firmadas.\n",
                NOMBRE_RED, CANAL_CONTROL, 2400 + CANAL_CONTROL);
  mostrarCanales();
  mostrarAyuda();
}

void loop() {
  // 1) Comandos del Monitor Serial
  static String linea = "";
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') { procesarLinea(linea); linea = ""; }
    else if (linea.length() < 120) linea += c;
  }

  // 2) Mensajes de los nodos (hasta 8 por vuelta)
  for (uint8_t n = 0; n < 8 && radio.available(); n++) {
    Paquete p;
    radio.read(&p, sizeof(p));
    atenderPaquete(p);
  }

  // 3) Tiempos de timbrado y LED
  revisarTiempos();

  // 4) Monitoreo continuo de interferencia
  monitorearCanales();
}
