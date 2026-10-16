/*
  CENTRAL TELEFONICA v6 (DHCP + Optimizada) - Topologia estrella con nRF24L01+ PA+LNA
  Asigna automaticamente el ID (1..254) a cada Nodo Telefono segun su MAC unica.
*/
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include "esp_system.h"

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

// ======================= PROTOCOLO (igual en central y nodos) =======================
#define ID_CENTRAL     0
#define ID_SIN_ASIGNAR 255
#define CANAL_CONTROL  80        // 2480 MHz: donde todos esperan y piden llamadas
#define CANAL_MIN      0
#define CANAL_MAX      83

enum Tipo : uint8_t {
  REGISTRO = 1, LLAMAR, CONTESTAR, RECHAZAR, LIBERAR,
  ENTRANTE, CONECTAR, RESPUESTA, TEXTO, KEEPALIVE, COLGAR,
  AVISO, VOZ, AUDIO, PEDIR_ID, ASIGNAR_ID
};
enum Codigo : uint8_t {
  R_TIMBRANDO = 1, R_OCUPADO, R_NO_DISPONIBLE, R_RECHAZADO,
  R_NO_CONTESTA, R_SIN_CANALES, R_CANCELADO
};
struct Paquete {          // 32 bytes exactos (maximo del nRF24)
  uint8_t tipo;
  uint8_t origen;
  uint8_t destino;
  uint8_t canal;
  uint8_t dato;
  uint8_t secuencia;
  char    texto[26];
};
static_assert(sizeof(Paquete) == 32, "El paquete debe medir 32 bytes");

// ============================ TABLAS DE LA CENTRAL ============================
#define MAX_NODOS       32
#define NODO_CADUCA_MS  15000UL   // sin registrarse en 15 s = desconectado
struct Nodo {
  uint8_t id;
  char mac[13];
  unsigned long visto;
};

#define MAX_LLAMADAS    16
#define TIMBRE_MAX_MS   20000UL
enum EstadoLlamada : uint8_t { LIBRE = 0, TIMBRANDO, EN_CURSO };
struct Llamada {
  EstadoLlamada estado;
  uint8_t origen, destino, canal;
  unsigned long desde;
};

// Monitoreo de interferencia
#define UMBRAL_RUIDO      25
#define MONITOREO_CADA_MS 40
#define MUESTRAS_MONITOR  4
#define MUESTRAS_VERIFICA 8
// ===================================================================================

void direccionDe(uint8_t id, uint8_t *dir) {
  dir[0] = id; dir[1] = 'N'; dir[2] = 'O'; dir[3] = 'D'; dir[4] = 'E';
}

// Direccion de pipe temporal basada en los ultimos 4 caracteres de la MAC del nodo
void direccionTempDeMac(const char *mac, uint8_t *dir) {
  dir[0] = ID_SIN_ASIGNAR;
  dir[1] = mac[8]  ? (uint8_t)mac[8]  : '0';
  dir[2] = mac[9]  ? (uint8_t)mac[9]  : '0';
  dir[3] = mac[10] ? (uint8_t)mac[10] : '0';
  dir[4] = mac[11] ? (uint8_t)mac[11] : '0';
}

RF24 radio(PIN_CE, PIN_CSN);
uint8_t secuencia = 0;

Nodo nodos[MAX_NODOS];
uint8_t numNodos = 0;
Llamada llamadas[MAX_LLAMADAS];
uint8_t ruido[CANAL_MAX + 1];
uint8_t canalMonitoreo = CANAL_MIN;
unsigned long tMonitoreo = 0;
bool monitoreoActivo = true;
uint32_t reiniciosRadio = 0;

// ------------------------------------------------------------------------------------
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

bool enviarEnCanal(uint8_t canalRF, uint8_t destino, uint8_t tipo,
                   uint8_t canal, uint8_t dato, const char *texto, uint8_t intentos = 1) {
  Paquete p = {};
  p.tipo = tipo; p.origen = ID_CENTRAL; p.destino = destino;
  p.canal = canal; p.dato = dato; p.secuencia = secuencia++;
  if (texto) strncpy(p.texto, texto, sizeof(p.texto) - 1);
  uint8_t dir[5]; direccionDe(destino, dir);

  radio.stopListening();
  if (canalRF != CANAL_CONTROL) radio.setChannel(canalRF);
  radio.openWritingPipe(dir);

  bool ok = false;
  for (uint8_t k = 0; k < intentos; k++) {
    if (radio.write(&p, sizeof(p))) { ok = true; break; }
    if (k + 1 < intentos) delayMicroseconds(1500);
  }

  if (canalRF != CANAL_CONTROL) radio.setChannel(CANAL_CONTROL);
  radio.startListening();
  return ok;
}

bool enviar(uint8_t destino, uint8_t tipo, uint8_t canal, uint8_t dato, uint8_t intentos = 1) {
  return enviarEnCanal(CANAL_CONTROL, destino, tipo, canal, dato, nullptr, intentos);
}

// Envia el ID asignado a un nodo usando su direccion temporal (basada en MAC) o su direccion actual
bool enviarAsignacionId(const uint8_t *dirPipe, uint8_t nuevoId, const char *mac) {
  Paquete p = {};
  p.tipo = ASIGNAR_ID;
  p.origen = ID_CENTRAL;
  p.destino = nuevoId;
  p.canal = CANAL_CONTROL;
  p.dato = nuevoId;
  p.secuencia = secuencia++;
  strncpy(p.texto, mac, sizeof(p.texto) - 1);

  radio.stopListening();
  radio.openWritingPipe(dirPipe);
  bool ok = false;
  for (uint8_t k = 0; k < 3; k++) {
    if (radio.write(&p, sizeof(p))) { ok = true; break; }
    delayMicroseconds(1500);
  }
  radio.startListening();
  return ok;
}

// ---- Gestion de Nodos y Asignacion de IDs (DHCP) ----
int buscarNodo(uint8_t id) {
  for (int i = 0; i < numNodos; i++) if (nodos[i].id == id) return i;
  return -1;
}

int buscarNodoPorMac(const char *mac) {
  if (!mac || strlen(mac) < 6) return -1;
  for (int i = 0; i < numNodos; i++) {
    if (strncmp(nodos[i].mac, mac, 12) == 0) return i;
  }
  return -1;
}

uint8_t buscarIdLibre() {
  for (uint8_t id = 1; id < ID_SIN_ASIGNAR; id++) {
    if (buscarNodo(id) < 0) return id;
  }
  return 0;
}

bool nodoEnLinea(uint8_t id) {
  int i = buscarNodo(id);
  return i >= 0 && millis() - nodos[i].visto < NODO_CADUCA_MS;
}

// Asigna o recupera el ID correspondiente a una MAC
uint8_t obtenerOAsignarId(const char *mac, uint8_t idPreferido = 0) {
  int idxMac = buscarNodoPorMac(mac);
  if (idxMac >= 0) {
    nodos[idxMac].visto = millis();
    return nodos[idxMac].id;
  }

  // Si no existe la MAC, buscar espacio en la tabla
  if (numNodos >= MAX_NODOS) {
    // Reemplazar un nodo desconectado si la tabla se lleno
    for (int i = 0; i < numNodos; i++) {
      if (!nodoEnLinea(nodos[i].id) && llamadaDe(nodos[i].id) < 0) {
        strncpy(nodos[i].mac, mac, 12);
        nodos[i].mac[12] = '\0';
        nodos[i].visto = millis();
        Serial.printf("Nodo %u reasignado a MAC %s\n", nodos[i].id, nodos[i].mac);
        return nodos[i].id;
      }
    }
    return 0;
  }

  uint8_t nuevoId = 0;
  if (idPreferido >= 1 && idPreferido < ID_SIN_ASIGNAR && buscarNodo(idPreferido) < 0) {
    nuevoId = idPreferido;
  } else {
    nuevoId = buscarIdLibre();
  }
  if (nuevoId == 0) return 0;

  int i = numNodos++;
  nodos[i].id = nuevoId;
  strncpy(nodos[i].mac, mac, 12);
  nodos[i].mac[12] = '\0';
  nodos[i].visto = millis();
  Serial.printf("Nodo %u asignado a MAC %s\n", nuevoId, nodos[i].mac);
  return nuevoId;
}

void atenderPedirId(const Paquete &p) {
  char mac[13] = {};
  strncpy(mac, p.texto, 12);
  if (strlen(mac) < 6) return;

  uint8_t id = obtenerOAsignarId(mac, 0);
  if (id == 0) return;

  uint8_t dirTemp[5];
  direccionTempDeMac(mac, dirTemp);
  delayMicroseconds(800); // Dar tiempo a que el nodo pase a escucha
  bool ok = enviarAsignacionId(dirTemp, id, mac);
  Serial.printf("Solicitud de ID desde MAC %s -> Asignado Nodo %u (%s)\n",
                mac, id, ok ? "entregado" : "sin ACK");
}

// Verifica y actualiza el registro de un nodo activo
uint8_t verificarRegistroNodo(uint8_t idOrigen, const char *textoMac) {
  if (idOrigen == ID_CENTRAL || idOrigen == ID_SIN_ASIGNAR) return 0;

  char mac[13] = {};
  if (textoMac && strlen(textoMac) >= 6) strncpy(mac, textoMac, 12);

  if (mac[0] != '\0') {
    int idxId = buscarNodo(idOrigen);
    int idxMac = buscarNodoPorMac(mac);

    // Si el ID ya lo tiene OTRA MAC distinta, o esta MAC tenia OTRO ID: corregir al nodo
    if ((idxId >= 0 && idxId != idxMac && nodos[idxId].mac[0] != '\0') ||
        (idxMac >= 0 && nodos[idxMac].id != idOrigen)) {
      uint8_t idCorrecto = obtenerOAsignarId(mac, 0);
      if (idCorrecto > 0) {
        uint8_t dirActual[5];
        direccionDe(idOrigen, dirActual);
        delayMicroseconds(800);
        enviarAsignacionId(dirActual, idCorrecto, mac);
        Serial.printf("Conflicto de ID resuelto para MAC %s: %u -> %u\n", mac, idOrigen, idCorrecto);
      }
      return idCorrecto;
    }

    // Si el nodo ya estaba con ese ID pero no teniamos su MAC guardada
    if (idxId >= 0 && nodos[idxId].mac[0] == '\0') {
      strncpy(nodos[idxId].mac, mac, 12);
      nodos[idxId].mac[12] = '\0';
      nodos[idxId].visto = millis();
      return idOrigen;
    }

    return obtenerOAsignarId(mac, idOrigen);
  }

  // Fallback para nodos antiguos que no envian MAC en REGISTRO
  int i = buscarNodo(idOrigen);
  if (i < 0) {
    if (numNodos >= MAX_NODOS) return 0;
    i = numNodos++;
    nodos[i].id = idOrigen;
    nodos[i].mac[0] = '\0';
    Serial.printf("Nodo %u registrado (sin MAC)\n", idOrigen);
  }
  nodos[i].visto = millis();
  return idOrigen;
}

// ---- Llamadas ----
int llamadaDe(uint8_t id) {
  for (int i = 0; i < MAX_LLAMADAS; i++)
    if (llamadas[i].estado != LIBRE &&
        (llamadas[i].origen == id || llamadas[i].destino == id)) return i;
  return -1;
}
int llamadaEnCanal(uint8_t c) {
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

void monitorearCanales() {
  if (!monitoreoActivo) return;
  if (millis() - tMonitoreo < MONITOREO_CADA_MS) return;
  tMonitoreo = millis();

  uint8_t c = canalMonitoreo;
  canalMonitoreo = (canalMonitoreo >= CANAL_MAX) ? CANAL_MIN : canalMonitoreo + 1;
  if (c == CANAL_CONTROL || llamadaEnCanal(c) >= 0) return;

  uint8_t medida = medirCanal(c, MUESTRAS_MONITOR);
  ruido[c] = (ruido[c] * 3 + medida) / 4;
}

int distanciaAOcupados(int c) {
  int d = abs(c - CANAL_CONTROL);
  for (int i = 0; i < MAX_LLAMADAS; i++)
    if (llamadas[i].estado != LIBRE) d = min(d, abs(c - (int)llamadas[i].canal));
  return d;
}

int elegirCanal() {
  for (int pasada = 0; pasada < 2; pasada++) {
    int mejor = -1, mejorDist = -1, mejorRuido = 999;
    for (int c = CANAL_MIN; c <= CANAL_MAX; c++) {
      if (c == CANAL_CONTROL || llamadaEnCanal(c) >= 0) continue;
      if (pasada == 0 && ruido[c] > UMBRAL_RUIDO) continue;
      int dist = min(distanciaAOcupados(c), 6);
      if (dist > mejorDist || (dist == mejorDist && ruido[c] < mejorRuido)) {
        mejor = c; mejorDist = dist; mejorRuido = ruido[c];
      }
    }
    if (mejor >= 0) return mejor;
  }
  return -1;
}

int elegirCanalVerificado() {
  for (int intento = 0; intento < 3; intento++) {
    int c = elegirCanal();
    if (c < 0) return -1;
    uint8_t medida = medirCanal(c, MUESTRAS_VERIFICA);
    ruido[c] = max(ruido[c], medida);
    if (medida <= UMBRAL_RUIDO) return c;
    Serial.printf("  Canal %d tiene interferencia (%u%%), buscando otro...\n", c, medida);
  }
  return elegirCanal();
}

// ------------------------------------------------------------------------------------
void atenderLlamar(uint8_t a, uint8_t b) {
  Serial.printf("Nodo %u quiere llamar al nodo %u\n", a, b);
  if (b == a || b == ID_CENTRAL || b == ID_SIN_ASIGNAR || !nodoEnLinea(b)) {
    enviar(a, RESPUESTA, 0, R_NO_DISPONIBLE, 2); return;
  }
  if (llamadaDe(a) >= 0 || llamadaDe(b) >= 0) {
    enviar(a, RESPUESTA, 0, R_OCUPADO, 2); return;
  }
  int i = espacioLlamada();
  int canal = (i >= 0) ? elegirCanalVerificado() : -1;
  if (canal < 0) { enviar(a, RESPUESTA, 0, R_SIN_CANALES, 2); return; }

  llamadas[i] = Llamada{TIMBRANDO, a, b, (uint8_t)canal, millis()};
  if (!enviar(b, ENTRANTE, 0, a, 2)) {
    liberar(i, "el destino no respondio");
    enviar(a, RESPUESTA, 0, R_NO_DISPONIBLE, 2);
    return;
  }
  enviar(a, RESPUESTA, 0, R_TIMBRANDO, 2);
  Serial.printf("  Timbrando en nodo %u (canal %d reservado, ruido %u%%)\n",
                b, canal, ruido[canal]);
}

void atenderContestar(uint8_t b) {
  int i = llamadaDe(b);
  if (i < 0 || llamadas[i].estado != TIMBRANDO || llamadas[i].destino != b) return;
  uint8_t a = llamadas[i].origen, canal = llamadas[i].canal;

  delayMicroseconds(1000);
  bool okB = enviar(b, CONECTAR, canal, a, 3);
  bool okA = enviar(a, CONECTAR, canal, b, 3);

  if (okA && okB) {
    llamadas[i].estado = EN_CURSO;
    llamadas[i].desde = millis();
    Serial.printf("CONECTADOS %u <-> %u en canal %u (%u MHz)\n", a, b, canal, 2400 + canal);
  } else {
    if (okB) enviarEnCanal(canal, b, COLGAR, 0, 0, nullptr, 3);
    if (okA) enviarEnCanal(canal, a, COLGAR, 0, 0, nullptr, 3);
    enviar(a, RESPUESTA, 0, R_NO_DISPONIBLE, 2);
    enviar(b, RESPUESTA, 0, R_CANCELADO, 2);
    liberar(i, "fallo al sincronizar cambio de canal");
  }
}

void atenderRechazar(uint8_t b) {
  int i = llamadaDe(b);
  if (i < 0 || llamadas[i].estado != TIMBRANDO) return;
  enviar(llamadas[i].origen, RESPUESTA, 0, R_RECHAZADO, 2);
  liberar(i, "rechazada");
}

void atenderLiberar(uint8_t x) {
  int i = llamadaDe(x);
  if (i < 0) return;
  uint8_t otro = (llamadas[i].origen == x) ? llamadas[i].destino : llamadas[i].origen;
  if (llamadas[i].estado == TIMBRANDO) {
    enviar(otro, RESPUESTA, 0, R_CANCELADO, 2);
    liberar(i, "cancelada antes de contestar");
  } else {
    enviarEnCanal(llamadas[i].canal, otro, COLGAR, 0, 0, nullptr, 2);
    liberar(i, "colgaron");
  }
}

void revisarTiempos() {
  bool alguna = false;
  for (int i = 0; i < MAX_LLAMADAS; i++) {
    if (llamadas[i].estado == TIMBRANDO && millis() - llamadas[i].desde > TIMBRE_MAX_MS) {
      enviar(llamadas[i].origen, RESPUESTA, 0, R_NO_CONTESTA, 2);
      enviar(llamadas[i].destino, RESPUESTA, 0, R_CANCELADO, 2);
      liberar(i, "no contesto");
    }
    if (llamadas[i].estado == EN_CURSO) alguna = true;
  }
  static int ledAnterior = -1;
  if (alguna != ledAnterior) { ledAnterior = alguna; ledModo(alguna ? 2 : 1); }
}

// ------------------------------------------------------------------------------------
bool entregarAviso(uint8_t id, const String &texto) {
  int l = llamadaDe(id);
  uint8_t canalRF = (l >= 0 && llamadas[l].estado == EN_CURSO) ? llamadas[l].canal : CANAL_CONTROL;
  bool ok = true;
  for (unsigned int i = 0; i < texto.length(); i += 25) {
    String trozo = texto.substring(i, i + 25);
    ok = enviarEnCanal(canalRF, id, AVISO, 0, 0, trozo.c_str(), 2) && ok;
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
void mostrarTabla() {
  Serial.println("--- Nodos registrados (DHCP activo) ---");
  if (numNodos == 0) Serial.println("  (ninguno)");
  for (int i = 0; i < numNodos; i++) {
    uint8_t id = nodos[i].id;
    const char *mac = nodos[i].mac[0] ? nodos[i].mac : "sin-MAC";
    int l = llamadaDe(id);
    if (l >= 0) {
      uint8_t otro = llamadas[l].origen == id ? llamadas[l].destino : llamadas[l].origen;
      Serial.printf("  Nodo %3u [%s] : %s con %u (canal %u)\n", id, mac,
                    llamadas[l].estado == TIMBRANDO ? "timbrando" : "en llamada",
                    otro, llamadas[l].canal);
    } else {
      Serial.printf("  Nodo %3u [%s] : %s\n", id, mac, nodoEnLinea(id) ? "libre" : "desconectado");
    }
  }
}

void mostrarCanales() {
  Serial.printf("--- Canales %u a %u (%u a %u MHz) ---\n", CANAL_MIN, CANAL_MAX,
                2400 + CANAL_MIN, 2400 + CANAL_MAX);
  Serial.print("  ");
  for (int c = CANAL_MIN; c <= CANAL_MAX; c++) Serial.print(c % 10 == 0 ? (char)('0' + (c / 10) % 10) : ' ');
  Serial.println();
  int libres = 0, conRuido = 0, enUso = 0;
  Serial.print("  ");
  for (int c = CANAL_MIN; c <= CANAL_MAX; c++) {
    char s;
    if (c == CANAL_CONTROL)              s = 'C';
    else if (llamadaEnCanal(c) >= 0)   { s = 'L'; enUso++; }
    else if (ruido[c] > UMBRAL_RUIDO)  { s = 'x'; conRuido++; }
    else                               { s = '.'; libres++; }
    Serial.print(s);
  }
  Serial.println();
  Serial.println("  .=libre  L=llamada  C=control  x=interferencia (decenas arriba: 0=0-9, 1=10-19...)");
  Serial.printf("  Libres: %d   En llamada: %d   Con interferencia: %d\n", libres, enUso, conRuido);

  bool alguna = false;
  for (int i = 0; i < MAX_LLAMADAS; i++) {
    if (llamadas[i].estado == LIBRE) continue;
    if (!alguna) { Serial.println("  Llamadas:"); alguna = true; }
    Serial.printf("    canal %2u (%u MHz): %u -> %u  %s  %lu s\n", llamadas[i].canal,
                  2400 + llamadas[i].canal, llamadas[i].origen, llamadas[i].destino,
                  llamadas[i].estado == TIMBRANDO ? "timbrando" : "EN CURSO",
                  (millis() - llamadas[i].desde) / 1000);
  }
}

void mostrarRuido() {
  Serial.println("--- Interferencia medida por canal (promedio) ---");
  bool alguno = false;
  for (int c = CANAL_MIN; c <= CANAL_MAX; c++) {
    if (ruido[c] == 0 || c == CANAL_CONTROL) continue;
    alguno = true;
    Serial.printf("  canal %2d (%u MHz): %3u%% ", c, 2400 + c, ruido[c]);
    for (int k = 0; k < ruido[c] / 4; k++) Serial.print('#');
    Serial.println(ruido[c] > UMBRAL_RUIDO ? "  <- se evita" : "");
  }
  if (!alguno) Serial.println("  Ningun canal con interferencia detectada.");
}

void mostrarAyuda() {
  Serial.println("Comandos: /todos texto | /msg ID texto | /tabla | /canales | /ruido | /monitor | /estado | /ayuda");
}

const char *motivoReinicio() {
  switch (esp_reset_reason()) {
    case ESP_RST_POWERON:  return "encendido normal";
    case ESP_RST_SW:       return "reinicio por software";
    case ESP_RST_PANIC:    return "ERROR del programa (panic)";
    case ESP_RST_INT_WDT:
    case ESP_RST_TASK_WDT:
    case ESP_RST_WDT:      return "WATCHDOG (el programa se trabo)";
    case ESP_RST_BROWNOUT: return "FALTA DE CORRIENTE (brownout): revisa el capacitor y el cable USB";
    case ESP_RST_EXT:      return "boton de reset / puerto USB";
    default:               return "otro";
  }
}

void mostrarEstado() {
  int enLinea = 0, activas = 0;
  for (int i = 0; i < numNodos; i++) if (nodoEnLinea(nodos[i].id) || llamadaDe(nodos[i].id) >= 0) enLinea++;
  for (int i = 0; i < MAX_LLAMADAS; i++) if (llamadas[i].estado != LIBRE) activas++;
  Serial.printf("Central (%s) | %d nodos conectados | %d llamadas | monitoreo %s | "
                "encendida hace %lu s | ultimo reinicio: %s | reinicios del nRF: %lu\n",
                PLACA, enLinea, activas, monitoreoActivo ? "ON" : "OFF",
                millis() / 1000, motivoReinicio(), (unsigned long)reiniciosRadio);
}

void procesarLinea(String l) {
  l.trim();
  if (l.length() == 0) return;
  if (l.startsWith("/todos ")) {
    String t = l.substring(7); t.trim();
    Serial.printf("Mensaje a todos: \"%s\"\n", t.c_str());
    avisoATodos(t);
  }
  else if (l.startsWith("/msg ")) {
    String resto = l.substring(5); resto.trim();
    int esp = resto.indexOf(' ');
    int id = resto.substring(0, esp).toInt();
    if (esp < 0 || id < 1 || id > 254) { Serial.println("Uso: /msg ID texto"); return; }
    bool ok = entregarAviso(id, resto.substring(esp + 1));
    Serial.printf("Mensaje al nodo %d: %s\n", id, ok ? "entregado" : "NO RESPONDIO");
  }
  else if (l == "/tabla")   mostrarTabla();
  else if (l == "/canales") mostrarCanales();
  else if (l == "/ruido")   mostrarRuido();
  else if (l == "/estado")  mostrarEstado();
  else if (l == "/monitor") {
    monitoreoActivo = !monitoreoActivo;
    Serial.printf("Monitoreo de canales %s\n", monitoreoActivo ? "ENCENDIDO" : "APAGADO");
  }
  else mostrarAyuda();
}

// ------------------------------------------------------------------------------------
void configurarRadio() {
  radio.setChannel(CANAL_CONTROL);
  radio.setDataRate(RF24_1MBPS);
  radio.setPALevel(RF24_PA_LOW);
  radio.setPayloadSize(sizeof(Paquete));
  radio.setAutoAck(true);
  radio.enableDynamicAck();
  radio.setRetries(5, 15);
  uint8_t dir[5]; direccionDe(ID_CENTRAL, dir);
  radio.openReadingPipe(1, dir);
  radio.startListening();
}

void revisarRadio() {
  static unsigned long t = 0;
  if (millis() - t < 500) return;
  t = millis();
  if (radio.getChannel() != CANAL_CONTROL) {
    reiniciosRadio++;
    Serial.printf("AVISO: el nRF24 se reinicio solo (#%lu). Reconfigurando...\n",
                  (unsigned long)reiniciosRadio);
    radio.begin(&SPI);
    configurarRadio();
  }
}

void setup() {
  Serial.begin(115200);
#if ARDUINO_USB_CDC_ON_BOOT
  Serial.setTxTimeoutMs(0);
#endif
  delay(1500);
  Serial.printf("\nUltimo reinicio: %s\n", motivoReinicio());
#ifdef PIN_LED
  pinMode(PIN_LED, OUTPUT);
#endif
  ledModo(0);

  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CSN);
  if (!radio.begin(&SPI)) {
    Serial.println("ERROR: el nRF24L01 no responde. Revisa cables y alimentacion.");
    while (true) { ledModo(3); delay(150); ledModo(0); delay(150); }
  }
  configurarRadio();

  Serial.println("\nMidiendo interferencia en todos los canales...");
  for (int c = CANAL_MIN; c <= CANAL_MAX; c++)
    ruido[c] = (c == CANAL_CONTROL) ? 0 : medirCanal(c, MUESTRAS_VERIFICA);

  radio.setChannel(CANAL_CONTROL);
  radio.startListening();
  ledModo(1);

  Serial.printf("=== CENTRAL TELEFONICA v6 (ID Automatico) lista (%s) ===\n", PLACA);
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

  // 2) Mensajes de los nodos
  while (radio.available()) {
    Paquete p;
    radio.read(&p, sizeof(p));
    p.texto[sizeof(p.texto) - 1] = '\0';

    if (p.tipo == PEDIR_ID || p.origen == ID_SIN_ASIGNAR) {
      atenderPedirId(p);
      continue;
    }

    uint8_t idReal = verificarRegistroNodo(p.origen, p.tipo == REGISTRO ? p.texto : nullptr);
    if (idReal == 0 || idReal != p.origen) continue;

    switch (p.tipo) {
      case REGISTRO: {
        int i = llamadaDe(p.origen);
        if (i >= 0 && millis() - llamadas[i].desde > 2500) {
          uint8_t otro = (llamadas[i].origen == p.origen) ? llamadas[i].destino : llamadas[i].origen;
          if (llamadas[i].estado == EN_CURSO) {
            enviarEnCanal(llamadas[i].canal, otro, COLGAR, 0, 0, nullptr, 2);
          } else {
            enviar(otro, RESPUESTA, 0, R_CANCELADO, 2);
          }
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

  // 3) Tiempos de timbrado y LED
  revisarTiempos();

  // 4) Monitoreo continuo de interferencia
  monitorearCanales();

  // 5) Salud del radio
  revisarRadio();
  delay(1);
}