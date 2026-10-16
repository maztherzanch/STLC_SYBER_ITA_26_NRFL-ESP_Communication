/*
  CENTRAL TELEFONICA v2 - Topologia estrella con nRF24L01+ PA+LNA
  ----------------------------------------------------------------
  Novedades de la v2:
   - La central puede mandar MENSAJES a todos los nodos (/todos) o a uno (/msg).
     Si un nodo esta en llamada, la central salta a su canal, se lo entrega
     y regresa al canal de control.
   - La central MONITOREA todos los canales: mide constantemente si hay
     interferencia (WiFi, Bluetooth, otros equipos) y solo asigna canales limpios.
   - Usa TODOS los canales de la banda ISM de 2.4 GHz: del 0 (2400 MHz)
     al 83 (2483 MHz). El 80 es el de control; quedan 83 para llamadas.

  Como funciona (igual que antes):
   1. Los nodos se registran con la central en el CANAL DE CONTROL.
   2. Un nodo pide llamar a otro; la central lo hace timbrar.
   3. Si contesta, la central elige el mejor canal libre (sin interferencia
      y lo mas lejos posible de los canales ocupados) y manda a los dos ahi.
   4. Al colgar, regresan al canal de control y el canal queda libre.

  Comandos en el Monitor Serial de la central:
     /todos texto     -> mensaje a TODOS los nodos (incluso los que estan en llamada)
     /msg ID texto    -> mensaje solo al nodo ID
     /tabla           -> nodos registrados y su estado
     /canales         -> mapa de los 84 canales y llamadas activas
     /ruido           -> canales con interferencia detectada (con porcentaje)
     /ayuda           -> esta lista

  PLACAS: el codigo detecta solo la placa elegida en Tools -> Board:
   a) ESP32-S3 SuperMini ("ESP32S3 Dev Module", USB CDC On Boot: Enabled)
        CE->9  CSN->10  MOSI->11  SCK->12  MISO->13   LED RGB de la placa (GPIO48)
   b) ESP32-C3 SuperMini ("ESP32C3 Dev Module", USB CDC On Boot: Enabled)
        CE->3  CSN->7  SCK->4  MOSI->6  MISO->5       LED de la placa (GPIO8)
   c) ESP32 DevKit normal ("ESP32 Dev Module")
        CE->2  CSN->4  SCK->18  MISO->19  MOSI->23    LED externo en GPIO12
   En todas: VCC->3V3, GND->GND, capacitor 10-100uF entre VCC y GND del nRF.
   Monitor Serial a 115200 con "Nueva linea".
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

// ======================= PROTOCOLO (igual en central y nodos) =======================
#define ID_CENTRAL     0
#define CANAL_CONTROL  80        // 2480 MHz: donde todos esperan y piden llamadas

// Rango de canales que la central puede asignar a las llamadas.
// 0..83 = 2400..2483 MHz = toda la banda ISM de 2.4 GHz (uso libre).
// El nRF24 llega hasta el 125 (2525 MHz), pero del 84 en adelante ya esta
// FUERA de la banda de uso libre, por eso no se usan.
#define CANAL_MIN      0
#define CANAL_MAX      83

enum Tipo : uint8_t {
  REGISTRO = 1,  // nodo -> central : "estoy aqui y libre" (cada 5 s)
  LLAMAR,        // nodo -> central : quiero llamar a 'dato'
  CONTESTAR,     // nodo -> central : acepto la llamada entrante
  RECHAZAR,      // nodo -> central : rechazo la llamada entrante
  LIBERAR,       // nodo -> central : colgue / cancelo
  ENTRANTE,      // central -> nodo : te llama 'dato'
  CONECTAR,      // central -> nodo : vete al canal 'canal' con la pareja 'dato'
  RESPUESTA,     // central -> nodo : aviso (codigo en 'dato')
  TEXTO,         // nodo <-> nodo   : mensaje dentro de la llamada
  KEEPALIVE,     // nodo <-> nodo   : "sigo aqui" durante la llamada
  COLGAR,        // nodo <-> nodo   : termino la llamada
  AVISO          // central -> nodo : mensaje de la central (NUEVO en v2)
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

// ============================ TABLAS DE LA CENTRAL ============================
#define MAX_NODOS       32
#define NODO_CADUCA_MS  15000UL   // sin registrarse en 15 s = desconectado
struct Nodo { uint8_t id; unsigned long visto; };

#define MAX_LLAMADAS    16        // llamadas simultaneas (con 32 nodos, maximo 16)
#define TIMBRE_MAX_MS   20000UL   // 20 s timbrando sin contestar -> "no contesta"
enum EstadoLlamada : uint8_t { LIBRE = 0, TIMBRANDO, EN_CURSO };
struct Llamada {
  EstadoLlamada estado;
  uint8_t origen, destino, canal;
  unsigned long desde;
};

// Monitoreo de interferencia
#define UMBRAL_RUIDO      25      // % de muestras con senal a partir del cual el canal se evita
#define MONITOREO_CADA_MS 40      // cada 40 ms se revisa un canal (barrido completo ~3.4 s)
#define MUESTRAS_MONITOR  4       // muestras por revision periodica
#define MUESTRAS_VERIFICA 20      // muestras al verificar un canal antes de asignarlo
// ===================================================================================

void direccionDe(uint8_t id, uint8_t *dir) {   // direccion de 5 bytes de cada nodo
  dir[0] = id; dir[1] = 'N'; dir[2] = 'O'; dir[3] = 'D'; dir[4] = 'E';
}

RF24 radio(PIN_CE, PIN_CSN);
uint8_t secuencia = 0;

Nodo nodos[MAX_NODOS];
uint8_t numNodos = 0;
Llamada llamadas[MAX_LLAMADAS];
uint8_t ruido[CANAL_MAX + 1];         // % de ocupacion externa medida en cada canal
uint8_t canalMonitoreo = CANAL_MIN;
unsigned long tMonitoreo = 0;

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
  digitalWrite(PIN_LED, modo >= 2 ? LED_ON : !LED_ON);   // LED simple: encendido si hay llamadas
#endif
}

// ------------------------------------------------------------------------------------
// Envia un paquete en el canal indicado y regresa al canal de control.
bool enviarEnCanal(uint8_t canalRF, uint8_t destino, uint8_t tipo,
                   uint8_t canal, uint8_t dato, const char *texto) {
  Paquete p = {};
  p.tipo = tipo; p.origen = ID_CENTRAL; p.destino = destino;
  p.canal = canal; p.dato = dato; p.secuencia = secuencia++;
  if (texto) strncpy(p.texto, texto, sizeof(p.texto) - 1);
  uint8_t dir[5]; direccionDe(destino, dir);

  radio.stopListening();
  if (canalRF != CANAL_CONTROL) radio.setChannel(canalRF);
  radio.openWritingPipe(dir);
  bool ok = radio.write(&p, sizeof(p));
  if (canalRF != CANAL_CONTROL) radio.setChannel(CANAL_CONTROL);
  radio.startListening();
  return ok;
}

bool enviar(uint8_t destino, uint8_t tipo, uint8_t canal, uint8_t dato) {
  return enviarEnCanal(CANAL_CONTROL, destino, tipo, canal, dato, nullptr);
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
int llamadaDe(uint8_t id) {             // llamada donde participa 'id', o -1
  for (int i = 0; i < MAX_LLAMADAS; i++)
    if (llamadas[i].estado != LIBRE &&
        (llamadas[i].origen == id || llamadas[i].destino == id)) return i;
  return -1;
}
int llamadaEnCanal(uint8_t c) {         // llamada que ocupa el canal c, o -1
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
// Se toman varias muestras y se cuenta en cuantas hubo senal.
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

// Revisa un canal a la vez, en ciclo, para no descuidar el canal de control.
void monitorearCanales() {
  if (millis() - tMonitoreo < MONITOREO_CADA_MS) return;
  tMonitoreo = millis();

  // Saltar el canal de control y los que estan en uso por nuestras llamadas
  // (ahi la "senal" son nuestros propios nodos, no interferencia).
  uint8_t c = canalMonitoreo;
  canalMonitoreo = (canalMonitoreo >= CANAL_MAX) ? CANAL_MIN : canalMonitoreo + 1;
  if (c == CANAL_CONTROL || llamadaEnCanal(c) >= 0) return;

  uint8_t medida = medirCanal(c, MUESTRAS_MONITOR);
  ruido[c] = (ruido[c] * 3 + medida) / 4;      // promedio movil: suaviza picos aislados
}

// Que tan lejos esta el canal c del canal ocupado mas cercano (control o llamadas)
int distanciaAOcupados(int c) {
  int d = abs(c - CANAL_CONTROL);
  for (int i = 0; i < MAX_LLAMADAS; i++)
    if (llamadas[i].estado != LIBRE) d = min(d, abs(c - (int)llamadas[i].canal));
  return d;
}

// Elige el mejor canal libre:
//  1) primero solo entre los que NO tienen interferencia;
//  2) entre ellos, el mas alejado de los canales ocupados (hasta 6 MHz de separacion);
//  3) a igual distancia, el de menos ruido.
// Si todos los libres tienen interferencia, usa el menos ruidoso.
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

// Elige un canal y lo mide en ese momento antes de asignarlo.
int elegirCanalVerificado() {
  for (int intento = 0; intento < 5; intento++) {
    int c = elegirCanal();
    if (c < 0) return -1;
    uint8_t medida = medirCanal(c, MUESTRAS_VERIFICA);
    ruido[c] = max(ruido[c], medida);
    if (medida <= UMBRAL_RUIDO) return c;
    Serial.printf("  Canal %d tiene interferencia (%u%%), buscando otro...\n", c, medida);
  }
  return elegirCanal();       // si todo esta ruidoso, el mejor que haya
}

// ------------------------------------------------------------------------------------
void atenderLlamar(uint8_t a, uint8_t b) {
  Serial.printf("Nodo %u quiere llamar al nodo %u\n", a, b);
  if (b == a || b == ID_CENTRAL || !nodoEnLinea(b)) {
    enviar(a, RESPUESTA, 0, R_NO_DISPONIBLE); return;
  }
  if (llamadaDe(a) >= 0 || llamadaDe(b) >= 0) {
    enviar(a, RESPUESTA, 0, R_OCUPADO); return;
  }
  int i = espacioLlamada();
  int canal = (i >= 0) ? elegirCanalVerificado() : -1;
  if (canal < 0) { enviar(a, RESPUESTA, 0, R_SIN_CANALES); return; }

  llamadas[i] = Llamada{TIMBRANDO, a, b, (uint8_t)canal, millis()};
  if (!enviar(b, ENTRANTE, 0, a)) {
    liberar(i, "el destino no respondio");
    enviar(a, RESPUESTA, 0, R_NO_DISPONIBLE);
    return;
  }
  enviar(a, RESPUESTA, 0, R_TIMBRANDO);
  Serial.printf("  Timbrando en nodo %u (canal %d reservado, ruido %u%%)\n",
                b, canal, ruido[canal]);
}

void atenderContestar(uint8_t b) {
  int i = llamadaDe(b);
  if (i < 0 || llamadas[i].estado != TIMBRANDO || llamadas[i].destino != b) return;
  uint8_t a = llamadas[i].origen, canal = llamadas[i].canal;
  llamadas[i].estado = EN_CURSO;
  llamadas[i].desde = millis();
  enviar(b, CONECTAR, canal, a);       // mandar a los dos a su propio canal
  enviar(a, CONECTAR, canal, b);
  Serial.printf("CONECTADOS %u <-> %u en canal %u (%u MHz)\n", a, b, canal, 2400 + canal);
}

void atenderRechazar(uint8_t b) {
  int i = llamadaDe(b);
  if (i < 0 || llamadas[i].estado != TIMBRANDO) return;
  enviar(llamadas[i].origen, RESPUESTA, 0, R_RECHAZADO);
  liberar(i, "rechazada");
}

void atenderLiberar(uint8_t x) {
  int i = llamadaDe(x);
  if (i < 0) return;
  if (llamadas[i].estado == TIMBRANDO) {
    uint8_t otro = (llamadas[i].origen == x) ? llamadas[i].destino : llamadas[i].origen;
    enviar(otro, RESPUESTA, 0, R_CANCELADO);
    liberar(i, "cancelada antes de contestar");
  } else {
    liberar(i, "colgaron");
  }
}

void revisarTiempos() {
  bool alguna = false;
  for (int i = 0; i < MAX_LLAMADAS; i++) {
    if (llamadas[i].estado == TIMBRANDO && millis() - llamadas[i].desde > TIMBRE_MAX_MS) {
      enviar(llamadas[i].origen, RESPUESTA, 0, R_NO_CONTESTA);
      enviar(llamadas[i].destino, RESPUESTA, 0, R_CANCELADO);
      liberar(i, "no contesto");
    }
    if (llamadas[i].estado == EN_CURSO) alguna = true;
  }
  static int ledAnterior = -1;          // solo actualizar el LED cuando cambia
  if (alguna != ledAnterior) { ledAnterior = alguna; ledModo(alguna ? 2 : 1); }
}

// ------------------------------------------------------------------------------------
// MENSAJES DE LA CENTRAL
// Entrega un texto a un nodo, en el canal donde este (control o su llamada).
// Los textos largos se parten en pedazos de 25 caracteres.
bool entregarAviso(uint8_t id, const String &texto) {
  int l = llamadaDe(id);
  uint8_t canalRF = (l >= 0 && llamadas[l].estado == EN_CURSO) ? llamadas[l].canal : CANAL_CONTROL;
  bool ok = true;
  for (unsigned int i = 0; i < texto.length(); i += 25) {
    String trozo = texto.substring(i, i + 25);
    ok = enviarEnCanal(canalRF, id, AVISO, 0, 0, trozo.c_str()) && ok;
  }
  return ok;
}

void avisoATodos(const String &texto) {
  int entregados = 0, total = 0;
  for (int i = 0; i < numNodos; i++) {
    uint8_t id = nodos[i].id;
    if (!nodoEnLinea(id) && llamadaDe(id) < 0) continue;   // desconectado
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
  Serial.println("--- Nodos ---");
  if (numNodos == 0) Serial.println("  (ninguno)");
  for (int i = 0; i < numNodos; i++) {
    uint8_t id = nodos[i].id;
    int l = llamadaDe(id);
    if (l >= 0) {
      uint8_t otro = llamadas[l].origen == id ? llamadas[l].destino : llamadas[l].origen;
      Serial.printf("  Nodo %3u : %s con %u (canal %u)\n", id,
                    llamadas[l].estado == TIMBRANDO ? "timbrando" : "en llamada",
                    otro, llamadas[l].canal);
    } else {
      Serial.printf("  Nodo %3u : %s\n", id, nodoEnLinea(id) ? "libre" : "desconectado");
    }
  }
}

void mostrarCanales() {
  Serial.printf("--- Canales %u a %u (%u a %u MHz) ---\n", CANAL_MIN, CANAL_MAX,
                2400 + CANAL_MIN, 2400 + CANAL_MAX);
  // Regla con las decenas
  Serial.print("  ");
  for (int c = CANAL_MIN; c <= CANAL_MAX; c++) Serial.print(c % 10 == 0 ? (char)('0' + (c / 10) % 10) : ' ');
  Serial.println();
  // Mapa: un caracter por canal
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
  Serial.println("Comandos: /todos texto | /msg ID texto | /tabla | /canales | /ruido | /ayuda");
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
  else mostrarAyuda();
}

// ------------------------------------------------------------------------------------
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

  // Barrido inicial de todos los canales para arrancar ya con el mapa de interferencia
  Serial.println("\nMidiendo interferencia en todos los canales...");
  for (int c = CANAL_MIN; c <= CANAL_MAX; c++)
    ruido[c] = (c == CANAL_CONTROL) ? 0 : medirCanal(c, MUESTRAS_VERIFICA);

  radio.setChannel(CANAL_CONTROL);
  radio.startListening();
  ledModo(1);

  Serial.printf("=== CENTRAL TELEFONICA v2 lista (%s) ===\n", PLACA);
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
  if (radio.available()) {
    Paquete p;
    radio.read(&p, sizeof(p));
    registrarNodo(p.origen);
    switch (p.tipo) {
      case REGISTRO: {
        // Nodo que dice estar libre pero la central lo tiene en llamada = se reinicio
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

  // 3) Tiempos de timbrado y LED
  revisarTiempos();

  // 4) Monitoreo continuo de interferencia (un canal cada 40 ms)
  monitorearCanales();
}
