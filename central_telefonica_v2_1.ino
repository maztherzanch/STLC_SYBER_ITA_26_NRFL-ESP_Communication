/*
  CENTRAL TELEFONICA v2.1 (propia) - Topologia estrella con nRF24L01+ PA+LNA
  --------------------------------------------------------------------------
  Es compatible con los nodos que ya funcionan con la central v2: NO cambia el
  protocolo (mismos tipos de paquete, misma direccion de 5 bytes, canal de
  control 80, mismo tamano de 32 bytes). Solo mejora lo que hace la central.

  La central nunca transporta audio: el audio va de nodo a nodo en el canal que
  la central asigna. Por eso la calidad del audio depende de ELEGIR BIEN ese
  canal y de que la senalizacion (timbrar, conectar) no falle.

  Mejoras respecto a la v2
   Calidad del canal de audio
    - Memoria de interferencia: ademas del promedio, guarda el "pico" reciente de
      cada canal y lo olvida despacio. Asi se evitan canales con WiFi intermitente
      que el promedio solo no detectaba.
    - No mide los canales vecinos a una llamada ni al de control: la propia senal
      de los nodos se colaba en el canal de al lado y lo marcaba como "ruidoso".
    - Prefiere canales fuera de los lobulos de WiFi 1, 6 y 11 (a igual ruido).
    - Mas muestras por revision periodica (6 en vez de 4).
   Senalizacion fiable
    - Los paquetes importantes (ENTRANTE, CONECTAR, RESPUESTA) se reintentan hasta
      3 veces con el MISMO numero de secuencia (sin duplicados nuevos).
    - Al contestar, la llamada solo pasa a "en curso" si LOS DOS nodos recibieron
      el canal; si no, se cancela limpiamente y se avisa.
    - Un LLAMAR repetido (reintento del nodo) ya no responde "ocupado" por error.
    - Se atienden hasta 8 paquetes por vuelta (antes 1).
   Comandos nuevos y correcciones
    - /liberar ID : fuerza el fin de una llamada trabada.
    - /potencia min|baja|alta|max : potencia de la central sin recompilar.
    - /msg sin texto ya no se interpreta mal.
    - /tabla muestra estadisticas (llamadas, envios fallidos, tiempo encendida).

  Comandos en el Monitor Serial (115200, "Nueva linea"):
     /todos texto | /msg ID texto | /tabla | /canales | /ruido
     /liberar ID  | /potencia nivel | /ayuda

  PLACAS (se detecta sola en Tools -> Board):
   a) ESP32-S3 SuperMini : CE->9  CSN->10 MOSI->11 SCK->12 MISO->13  LED RGB GPIO48
   b) ESP32-C3 SuperMini : CE->3  CSN->7  SCK->4   MOSI->6 MISO->5   LED GPIO8
   c) ESP32 DevKit       : CE->2  CSN->4  SCK->18  MISO->19 MOSI->23 LED GPIO12
   En todas: VCC->3V3, GND->GND, capacitor 10-100uF entre VCC y GND del nRF.
*/
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

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

// ======================= PROTOCOLO (NO CAMBIAR: igual en central y nodos) =======================
#define ID_CENTRAL     0
#define CANAL_CONTROL  80        // 2480 MHz: donde todos esperan y piden llamadas
#define CANAL_MIN      0         // 0..83 = 2400..2483 MHz (banda ISM de uso libre)
#define CANAL_MAX      83

enum Tipo : uint8_t {
  REGISTRO = 1, LLAMAR, CONTESTAR, RECHAZAR, LIBERAR,      // nodo -> central
  ENTRANTE, CONECTAR, RESPUESTA,                           // central -> nodo
  TEXTO, KEEPALIVE, COLGAR,                                // nodo <-> nodo
  AVISO,                                                   // central -> nodo
  VOZ                                                      // nodo <-> nodo (la central no lo usa)
};
enum Codigo : uint8_t {
  R_TIMBRANDO = 1, R_OCUPADO, R_NO_DISPONIBLE, R_RECHAZADO,
  R_NO_CONTESTA, R_SIN_CANALES, R_CANCELADO
};
struct Paquete {          // 32 bytes exactos (maximo del nRF24)
  uint8_t tipo, origen, destino, canal, dato, secuencia;
  char    texto[26];
};

// ============================ AJUSTES DE LA CENTRAL ============================
#define MAX_NODOS         32
#define NODO_CADUCA_MS    15000UL  // sin registrarse en 15 s = desconectado
#define MAX_LLAMADAS      16
#define TIMBRE_MAX_MS     20000UL  // 20 s timbrando sin contestar -> "no contesta"

#define REINTENTOS_ENVIO  3        // intentos para paquetes importantes
#define UMBRAL_RUIDO      25       // % a partir del cual un canal se evita
#define MONITOREO_CADA_MS 40       // un canal cada 40 ms (barrido completo ~3.4 s)
#define MUESTRAS_MONITOR  6        // muestras por revision periodica
#define MUESTRAS_VERIFICA 20       // muestras al verificar un canal antes de asignarlo
#define DECAE_PICO        2        // puntos que baja el pico por barrido (~1.5 min de olvido)
#define PENAL_WIFI        10       // puntos extra a canales dentro de WiFi 1/6/11
// ===================================================================================

struct Nodo { uint8_t id; unsigned long visto; };
enum EstadoLlamada : uint8_t { LIBRE = 0, TIMBRANDO, EN_CURSO };
struct Llamada { EstadoLlamada estado; uint8_t origen, destino, canal; unsigned long desde; };

void direccionDe(uint8_t id, uint8_t *dir) {
  dir[0] = id; dir[1] = 'N'; dir[2] = 'O'; dir[3] = 'D'; dir[4] = 'E';
}

RF24 radio(PIN_CE, PIN_CSN);
uint8_t secuencia = 0;

Nodo nodos[MAX_NODOS];
uint8_t numNodos = 0;
Llamada llamadas[MAX_LLAMADAS];

uint8_t ruido[CANAL_MAX + 1];          // promedio movil de ocupacion externa (%)
uint8_t pico[CANAL_MAX + 1];           // maximo reciente, decae despacio (%)
uint8_t canalMonitoreo = CANAL_MIN;
unsigned long tMonitoreo = 0;

unsigned long totalLlamadas = 0, enviosFallidos = 0, paquetesRx = 0;

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
// ENVIO
Paquete armar(uint8_t destino, uint8_t tipo, uint8_t canal, uint8_t dato, const char *texto) {
  Paquete p = {};
  p.tipo = tipo; p.origen = ID_CENTRAL; p.destino = destino;
  p.canal = canal; p.dato = dato; p.secuencia = secuencia++;
  if (texto) strncpy(p.texto, texto, sizeof(p.texto) - 1);
  return p;
}

// Transmite en el canal indicado y siempre regresa al canal de control.
bool transmitir(uint8_t canalRF, const Paquete &p) {
  uint8_t dir[5]; direccionDe(p.destino, dir);
  radio.stopListening();
  if (canalRF != CANAL_CONTROL) radio.setChannel(canalRF);
  radio.openWritingPipe(dir);
  bool ok = radio.write(&p, sizeof(p));
  if (canalRF != CANAL_CONTROL) radio.setChannel(CANAL_CONTROL);
  radio.startListening();
  if (!ok) enviosFallidos++;
  return ok;
}

bool enviarEnCanal(uint8_t canalRF, uint8_t destino, uint8_t tipo,
                   uint8_t canal, uint8_t dato, const char *texto) {
  return transmitir(canalRF, armar(destino, tipo, canal, dato, texto));
}
bool enviar(uint8_t destino, uint8_t tipo, uint8_t canal, uint8_t dato) {
  return enviarEnCanal(CANAL_CONTROL, destino, tipo, canal, dato, nullptr);
}

// Para paquetes importantes: reintenta con el MISMO paquete (misma secuencia) y una
// pausa aleatoria corta, por si el nodo estaba transmitiendo su registro justo entonces.
bool enviarSeguro(uint8_t destino, uint8_t tipo, uint8_t canal, uint8_t dato) {
  Paquete p = armar(destino, tipo, canal, dato, nullptr);
  for (uint8_t i = 0; i < REINTENTOS_ENVIO; i++) {
    if (transmitir(CANAL_CONTROL, p)) return true;
    delay(10 + random(20));
  }
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
void registrarNodo(uint8_t id) {
  int i = buscarNodo(id);
  if (i < 0) {
    if (numNodos >= MAX_NODOS) return;
    i = numNodos++;
    nodos[i].id = id;
    Serial.printf("Nodo %u registrado\n", id);
  }
  nodos[i].visto = millis();
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
// testRPD() dice si en el canal hay una senal mas fuerte que -64 dBm.
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

// Canal de control, canales de llamada y sus vecinos inmediatos: ahi la "senal"
// son nuestros propios nodos (o su fuga al canal de al lado), no interferencia.
bool cercaDeOcupado(int c) {
  for (int d = -1; d <= 1; d++) {
    int k = c + d;
    if (k < CANAL_MIN || k > CANAL_MAX) continue;
    if (k == CANAL_CONTROL || llamadaEnCanal(k) >= 0) return true;
  }
  return false;
}

void anotarMedida(uint8_t c, uint8_t medida) {
  ruido[c] = (ruido[c] * 3 + medida) / 4;                           // promedio movil
  if (medida >= pico[c]) pico[c] = medida;                          // el pico sube de golpe...
  else pico[c] = pico[c] > DECAE_PICO ? pico[c] - DECAE_PICO : 0;   // ...y baja despacio
}

void monitorearCanales() {
  if (millis() - tMonitoreo < MONITOREO_CADA_MS) return;
  tMonitoreo = millis();
  uint8_t c = canalMonitoreo;
  canalMonitoreo = (c >= CANAL_MAX) ? CANAL_MIN : c + 1;
  if (cercaDeOcupado(c)) return;
  anotarMedida(c, medirCanal(c, MUESTRAS_MONITOR));
}

// ---- Eleccion de canal ----
int ruidoEfectivo(int c) { return max((int)ruido[c], (int)pico[c] / 2); }

// WiFi 1, 6 y 11 (centros 2412, 2437, 2462 MHz, +-11 MHz) ocupan los canales nRF 1-23, 26-48 y 51-73.
bool enBandaWiFi(int c) { return (c >= 1 && c <= 23) || (c >= 26 && c <= 48) || (c >= 51 && c <= 73); }

int puntaje(int c) { return ruidoEfectivo(c) + (enBandaWiFi(c) ? PENAL_WIFI : 0); }   // menor = mejor

int distanciaAOcupados(int c) {
  int d = abs(c - CANAL_CONTROL);
  for (int i = 0; i < MAX_LLAMADAS; i++)
    if (llamadas[i].estado != LIBRE) d = min(d, (int)abs(c - (int)llamadas[i].canal));
  return d;
}

// 1) solo canales sin interferencia; 2) el mas alejado de los ocupados (hasta 6 MHz);
// 3) a igual distancia, el de mejor puntaje. Si no queda ninguno limpio, el menos malo.
int elegirCanal() {
  for (int pasada = 0; pasada < 2; pasada++) {
    int mejor = -1, mejorDist = -1, mejorPuntaje = 9999;
    for (int c = CANAL_MIN; c <= CANAL_MAX; c++) {
      if (c == CANAL_CONTROL || llamadaEnCanal(c) >= 0) continue;
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

// Elige un canal y lo mide en ese momento antes de asignarlo.
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
    // Mismo LLAMAR repetido por el nodo (reintento): no es "ocupado", ya esta timbrando.
    bool repetida = llamadas[previa].estado == TIMBRANDO &&
                    llamadas[previa].origen == a && llamadas[previa].destino == b;
    enviarSeguro(a, RESPUESTA, 0, repetida ? R_TIMBRANDO : R_OCUPADO);
    return;
  }
  if (llamadaDe(b) >= 0) { enviarSeguro(a, RESPUESTA, 0, R_OCUPADO); return; }

  int i = espacioLlamada();
  int canal = (i >= 0) ? elegirCanalVerificado() : -1;
  if (canal < 0) { enviarSeguro(a, RESPUESTA, 0, R_SIN_CANALES); return; }

  llamadas[i] = Llamada{TIMBRANDO, a, b, (uint8_t)canal, millis()};
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
  uint8_t a = llamadas[i].origen, canal = llamadas[i].canal;

  bool okB = enviarSeguro(b, CONECTAR, canal, a);
  bool okA = enviarSeguro(a, CONECTAR, canal, b);
  if (okA && okB) {
    llamadas[i].estado = EN_CURSO;
    llamadas[i].desde = millis();
    totalLlamadas++;
    Serial.printf("CONECTADOS %u <-> %u en canal %u (%u MHz)\n", a, b, canal, 2400 + canal);
    return;
  }
  // Solo uno recibio el canal: avisarle donde esta (su canal de llamada) y cancelar.
  Serial.printf("No se pudo conectar %u <-> %u (respondio A:%s B:%s)\n", a, b, okA ? "si" : "no", okB ? "si" : "no");
  if (okA) enviarEnCanal(canal, a, RESPUESTA, 0, R_CANCELADO, nullptr);
  else     enviarSeguro(a, RESPUESTA, 0, R_NO_DISPONIBLE);
  if (okB) enviarEnCanal(canal, b, RESPUESTA, 0, R_CANCELADO, nullptr);
  else     enviarSeguro(b, RESPUESTA, 0, R_CANCELADO);
  liberar(i, "fallo al conectar");
}

void atenderRechazar(uint8_t b) {
  int i = llamadaDe(b);
  if (i < 0 || llamadas[i].estado != TIMBRANDO) return;
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

// Comando /liberar: termina una llamada aunque los nodos no hayan colgado.
void forzarLiberar(uint8_t x) {
  int i = llamadaDe(x);
  if (i < 0) { Serial.printf("El nodo %u no esta en ninguna llamada.\n", x); return; }
  if (llamadas[i].estado == EN_CURSO) {      // mejor esfuerzo: pedirles que cuelguen en su canal
    enviarEnCanal(llamadas[i].canal, llamadas[i].origen,  COLGAR, 0, 0, nullptr);
    enviarEnCanal(llamadas[i].canal, llamadas[i].destino, COLGAR, 0, 0, nullptr);
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
// MENSAJES DE LA CENTRAL (textos largos se parten en pedazos de 25 caracteres)
bool entregarAviso(uint8_t id, const String &texto) {
  int l = llamadaDe(id);
  uint8_t canalRF = (l >= 0 && llamadas[l].estado == EN_CURSO) ? llamadas[l].canal : CANAL_CONTROL;
  bool ok = true;
  for (unsigned int i = 0; i < texto.length(); i += 25)
    ok = enviarEnCanal(canalRF, id, AVISO, 0, 0, texto.substring(i, i + 25).c_str()) && ok;
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
// INFORMACION
void mostrarTabla() {
  Serial.println("--- Nodos ---");
  if (numNodos == 0) Serial.println("  (ninguno)");
  for (int i = 0; i < numNodos; i++) {
    uint8_t id = nodos[i].id;
    int l = llamadaDe(id);
    if (l >= 0) {
      uint8_t otro = llamadas[l].origen == id ? llamadas[l].destino : llamadas[l].origen;
      Serial.printf("  Nodo %3u : %s con %u (canal %u)\n", id,
                    llamadas[l].estado == TIMBRANDO ? "timbrando" : "en llamada", otro, llamadas[l].canal);
    } else {
      Serial.printf("  Nodo %3u : %s\n", id, nodoEnLinea(id) ? "libre" : "desconectado");
    }
  }
  unsigned long s = millis() / 1000;
  Serial.printf("  Llamadas conectadas: %lu | envios sin confirmar: %lu | paquetes recibidos: %lu | encendida: %lu h %lu min\n",
                totalLlamadas, enviosFallidos, paquetesRx, s / 3600, (s / 60) % 60);
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
    else if (llamadaEnCanal(c) >= 0)         { s = 'L'; enUso++; }
    else if (ruidoEfectivo(c) > UMBRAL_RUIDO){ s = 'x'; conRuido++; }
    else                                     { s = '.'; libres++; }
    Serial.print(s);
  }
  Serial.println();
  Serial.println("  .=libre  L=llamada  C=control  x=interferencia (decenas arriba: 0=0-9, 1=10-19...)");
  Serial.printf("  Libres: %d   En llamada: %d   Con interferencia: %d\n", libres, enUso, conRuido);
  bool alguna = false;
  for (int i = 0; i < MAX_LLAMADAS; i++) {
    if (llamadas[i].estado == LIBRE) continue;
    if (!alguna) { Serial.println("  Llamadas:"); alguna = true; }
    Serial.printf("    canal %2u (%u MHz): %u -> %u  %s  %lu s\n", llamadas[i].canal, 2400 + llamadas[i].canal,
                  llamadas[i].origen, llamadas[i].destino,
                  llamadas[i].estado == TIMBRANDO ? "timbrando" : "EN CURSO", (millis() - llamadas[i].desde) / 1000);
  }
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
  Serial.println("Comandos: /todos texto | /msg ID texto | /tabla | /canales | /ruido | /liberar ID | /potencia min|baja|alta|max | /ayuda");
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
    if (esp < 0) { Serial.println("Uso: /msg ID texto"); return; }
    int id = resto.substring(0, esp).toInt();
    if (id < 1 || id > 254) { Serial.println("Uso: /msg ID texto  (ID de 1 a 254)"); return; }
    bool ok = entregarAviso(id, resto.substring(esp + 1));
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
  else if (l == "/tabla")   mostrarTabla();
  else if (l == "/canales") mostrarCanales();
  else if (l == "/ruido")   mostrarRuido();
  else mostrarAyuda();
}

// ------------------------------------------------------------------------------------
void atenderPaquete(const Paquete &p) {
  paquetesRx++;
  registrarNodo(p.origen);
  switch (p.tipo) {
    case REGISTRO: {
      // Un nodo que dice estar libre pero la central lo tiene en llamada se reinicio.
      int i = llamadaDe(p.origen);
      if (i >= 0 && llamadas[i].estado == EN_CURSO && millis() - llamadas[i].desde > 3000)
        liberar(i, "el nodo regreso sin colgar");
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
#ifdef PIN_LED
  pinMode(PIN_LED, OUTPUT);
#endif
  ledModo(0);

  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CSN);
  if (!radio.begin(&SPI)) {
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

  Serial.printf("=== CENTRAL TELEFONICA v2.1 lista (%s) ===\n", PLACA);
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

  // 2) Mensajes de los nodos (hasta 8 por vuelta para no dejar a nadie esperando)
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
