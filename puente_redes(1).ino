/*
  PUENTE ENTRE REDES - une la red propia (central v2.2, "ITA1", canal 76) con la red de
  la otra central ("NODE", canal 80) para que sus nodos puedan llamarse entre si.

  HARDWARE: UN ESP32 con DOS modulos nRF24L01 (comparten SCK/MOSI/MISO, cada uno con su CE y CSN).
    Radio A (red propia)   y   Radio B (red del companero)
    ESP32 DevKit : A: CE 2,  CSN 4   | B: CE 16, CSN 17 | SCK 18, MISO 19, MOSI 23
    ESP32-C3     : A: CE 3,  CSN 7   | B: CE 1,  CSN 10 | SCK 4,  MISO 5,  MOSI 6
    ESP32-S3     : A: CE 9,  CSN 10  | B: CE 1,  CSN 2  | SCK 12, MISO 13, MOSI 11  (confirma que existan en tu placa)
    VCC->3V3, GND->GND y un capacitor de 10-100uF en CADA modulo.

  COMO FUNCIONA
    El puente se registra como NODO en las dos centrales, con numeros "virtuales":
        el nodo N de la OTRA red aparece aqui como el nodo 100+N.
    Ejemplo: un nodo de la red propia marca 103 -> llama al nodo 3 de la red del companero.
             Un nodo del companero marca 101 -> llama al nodo 1 de la red propia.
    No hay que cambiar las centrales ni los nodos. Cuando suena la llamada, el puente llama
    al destino en SU red, espera a que conteste, contesta en la red del llamante y copia la
    voz en los dos sentidos (reescribiendo origen/destino de cada paquete).

  LIMITES
    - UNA llamada entre redes a la vez (mientras tanto, las demas reciben "no disponible").
    - Hasta 5 nodos por red alcanzables (limite de canales de recepcion del nRF24).
    - Los nodos que se quieran alcanzar tienen que estar en las listas NODOS_A / NODOS_B.
    - La central del companero vera 3 nodos nuevos (101, 102, 103...). Avisale antes.
    - Agrega un salto de radio: la voz pasa por el puente, con algo mas de retraso.

  Comandos por el Monitor Serial (115200): /estado | /colgar | /info
*/
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// ============================ PINES ============================
#if defined(CONFIG_IDF_TARGET_ESP32S3)
  #define PIN_SCK 12
  #define PIN_MISO 13
  #define PIN_MOSI 11
  #define PIN_CE_A 9
  #define PIN_CSN_A 10
  #define PIN_CE_B 1
  #define PIN_CSN_B 2
#elif defined(CONFIG_IDF_TARGET_ESP32C3)
  #define PIN_SCK 4
  #define PIN_MISO 5
  #define PIN_MOSI 6
  #define PIN_CE_A 3
  #define PIN_CSN_A 7
  #define PIN_CE_B 1
  #define PIN_CSN_B 10
#elif defined(CONFIG_IDF_TARGET_ESP32)
  #define PIN_SCK 18
  #define PIN_MISO 19
  #define PIN_MOSI 23
  #define PIN_CE_A 2
  #define PIN_CSN_A 4
  #define PIN_CE_B 16
  #define PIN_CSN_B 17
#else
  #error "Placa no soportada"
#endif

// ============================ CONFIGURACION ============================
// Lado 0 = A = red propia | Lado 1 = B = red del companero
static const char    *RED[2]     = {"ITA1", "NODE"};   // 4 letras de cada red
static const uint8_t  CONTROL[2] = {76, 80};           // canal de control de cada red

// Nodos REALES de cada red que se podran alcanzar desde la otra (maximo 5 por lista).
static const uint8_t NODOS_A[] = {1, 2, 3};            // nodos de la red propia
static const uint8_t NODOS_B[] = {1, 2, 3};            // nodos de la red del companero (confirma sus IDs)
#define OFFSET_VIRTUAL 100                              // el nodo N del otro lado se marca como 100+N
// =======================================================================

static_assert(sizeof(NODOS_A) <= 5 && sizeof(NODOS_B) <= 5, "Maximo 5 nodos por lista");
static const uint8_t *const NODOS[2]     = {NODOS_A, NODOS_B};
static const uint8_t        NUM_NODOS[2] = {sizeof(NODOS_A), sizeof(NODOS_B)};

#define ID_CENTRAL          0
#define REGISTRO_CADA_MS    5000UL
#define KEEPALIVE_CADA_MS   2000UL
#define ESPERA_MAX_MS       35000UL    // llamada que nunca se conecta
#define FALLOS_MAX          4          // keepalives sin respuesta = el nodo desaparecio

enum Tipo : uint8_t {
  REGISTRO = 1, LLAMAR, CONTESTAR, RECHAZAR, LIBERAR,
  ENTRANTE, CONECTAR, RESPUESTA, TEXTO, KEEPALIVE, COLGAR, AVISO, VOZ
};
enum Codigo : uint8_t {
  R_TIMBRANDO = 1, R_OCUPADO, R_NO_DISPONIBLE, R_RECHAZADO, R_NO_CONTESTA, R_SIN_CANALES, R_CANCELADO
};
struct Paquete { uint8_t tipo, origen, destino, canal, dato, secuencia; char texto[26]; };
static_assert(sizeof(Paquete) == 32, "El paquete debe medir 32 bytes");

RF24 radioA(PIN_CE_A, PIN_CSN_A);
RF24 radioB(PIN_CE_B, PIN_CSN_B);
RF24 *R[2] = {&radioA, &radioB};
uint8_t canalAct[2] = {0, 0};
uint8_t sec[2] = {0, 0};

// ------------------------------------------------------------------------------------
// Estado de la unica llamada entre redes
enum Fase : uint8_t { F_LIBRE, F_PIDIENDO, F_CONECTANDO, F_EN_CURSO };
struct Puente {
  Fase fase;
  uint8_t lx;                 // lado donde esta el llamante
  uint8_t real[2];            // nodo real que participa en cada lado
  uint8_t virt[2];            // id virtual del puente en cada lado
  bool conectado[2];
  uint8_t fallos[2];
  unsigned long desde, tKA;
} pt;

unsigned long tProxRegistro = 0;
bool centralOk[2] = {true, true};

// ------------------------------------------------------------------------------------
void dir(uint8_t lado, uint8_t id, uint8_t *d) {
  d[0] = id; d[1] = RED[lado][0]; d[2] = RED[lado][1]; d[3] = RED[lado][2]; d[4] = RED[lado][3];
}

bool esReal(uint8_t lado, uint8_t id) {
  for (uint8_t i = 0; i < NUM_NODOS[lado]; i++) if (NODOS[lado][i] == id) return true;
  return false;
}
// Un id virtual en 'lado' representa a un nodo del lado contrario
bool esVirtual(uint8_t lado, uint8_t v) {
  return v > OFFSET_VIRTUAL && esReal(1 - lado, v - OFFSET_VIRTUAL);
}

// ---- Envio ----
bool transmitir(uint8_t lado, const Paquete &p) {
  uint8_t d[5]; dir(lado, p.destino, d);
  RF24 &r = *R[lado];
  r.stopListening();
  r.openWritingPipe(d);
  bool ok = r.write(&p, sizeof(p));
  r.startListening();
  return ok;
}

Paquete armar(uint8_t lado, uint8_t desde, uint8_t destino, uint8_t tipo, uint8_t dato, const char *texto) {
  Paquete p = {};
  p.tipo = tipo; p.origen = desde; p.destino = destino;
  p.canal = canalAct[lado]; p.dato = dato; p.secuencia = sec[lado]++;
  if (texto) strncpy(p.texto, texto, sizeof(p.texto) - 1);
  return p;
}

// Mensaje del puente (con su id virtual) hacia la central de ese lado; reintenta 3 veces.
bool aCentral(uint8_t lado, uint8_t desde, uint8_t tipo, uint8_t dato) {
  Paquete p = armar(lado, desde, ID_CENTRAL, tipo, dato, nullptr);
  for (uint8_t i = 0; i < 3; i++) {
    if (transmitir(lado, p)) return true;
    delay(8 + random(16));
  }
  return false;
}

void txRaw(uint8_t lado, uint8_t destino, const uint8_t *b) {      // voz: sin ACK
  uint8_t d[5]; dir(lado, destino, d);
  RF24 &r = *R[lado];
  r.stopListening();
  r.openWritingPipe(d);
  r.write(b, 32, true);
  r.startListening();
}

void canalLado(uint8_t lado, uint8_t c) {
  R[lado]->stopListening();
  R[lado]->setChannel(c);
  R[lado]->startListening();
  canalAct[lado] = c;
}

// ------------------------------------------------------------------------------------
void registrar() {
  for (uint8_t S = 0; S < 2; S++) {
    bool ok = true;
    for (uint8_t i = 0; i < NUM_NODOS[1 - S]; i++)
      ok = aCentral(S, OFFSET_VIRTUAL + NODOS[1 - S][i], REGISTRO, 0) && ok;
    if (!ok && centralOk[S]) Serial.printf("[ERR] La central de la red %s no responde\n", RED[S]);
    if (ok && !centralOk[S]) Serial.printf("Central de la red %s en linea\n", RED[S]);
    centralOk[S] = ok;
  }
}

// Termina la llamada entre redes: cuelga/libera los dos lados y regresa los radios al control.
void terminar(const char *motivo, uint8_t ladoColgo = 255) {
  if (pt.fase == F_LIBRE) return;
  Serial.printf("Puente: llamada terminada (%s)\n", motivo);
  for (uint8_t S = 0; S < 2; S++) {
    if (pt.conectado[S]) {
      if (S != ladoColgo) transmitir(S, armar(S, pt.virt[S], pt.real[S], COLGAR, 0, nullptr));
      canalLado(S, CONTROL[S]);
      aCentral(S, pt.virt[S], LIBERAR, 0);
    } else if (S == pt.lx) {
      aCentral(S, pt.virt[S], RECHAZAR, 0);     // el llamante sigue timbrando: se rechaza
    } else {
      aCentral(S, pt.virt[S], LIBERAR, 0);      // cancela el intento de llamada al destino
    }
  }
  pt.fase = F_LIBRE;
  tProxRegistro = millis();                     // volver a registrarse enseguida
}

// ------------------------------------------------------------------------------------
// Una central nos avisa que alguien llama a un nodo virtual
void entrante(uint8_t S, uint8_t v, uint8_t r) {
  uint8_t Y = 1 - S, k = v - OFFSET_VIRTUAL;
  if (pt.fase != F_LIBRE) {
    if (pt.lx == S && pt.real[S] == r && pt.real[Y] == k) return;   // ENTRANTE repetido
    aCentral(S, v, RECHAZAR, 0);                                    // puente ocupado
    return;
  }
  if (!esReal(S, r)) {
    Serial.printf("Nodo %u (red %s) no esta en la lista: se rechaza\n", r, RED[S]);
    aCentral(S, v, RECHAZAR, 0);
    return;
  }
  pt = Puente{};
  pt.fase = F_PIDIENDO; pt.lx = S; pt.desde = millis();
  pt.real[S] = r;                  pt.real[Y] = k;
  pt.virt[S] = v;                  pt.virt[Y] = OFFSET_VIRTUAL + r;
  Serial.printf("Llamada entre redes: nodo %u (%s) -> nodo %u (%s)\n", r, RED[S], k, RED[Y]);
  if (!aCentral(Y, pt.virt[Y], LLAMAR, k)) {
    Serial.printf("[ERR] La central de la red %s no responde\n", RED[Y]);
    aCentral(S, v, RECHAZAR, 0);
    pt.fase = F_LIBRE;
  }
}

void conectar(uint8_t S, uint8_t v, uint8_t canalRF) {
  if (pt.fase == F_LIBRE || v != pt.virt[S] || pt.conectado[S]) return;
  uint8_t X = pt.lx;
  if (S != X) {                    // el llamado contesto en su red: contestamos en la del llamante
    if (pt.fase != F_PIDIENDO) return;
    canalLado(S, canalRF);
    pt.conectado[S] = true; pt.fase = F_CONECTANDO;
    if (!aCentral(X, pt.virt[X], CONTESTAR, 0)) terminar("no se pudo contestar en la red del llamante");
  } else {                         // la red del llamante nos conecto: ya hay audio en los dos lados
    if (pt.fase != F_CONECTANDO) return;
    canalLado(S, canalRF);
    pt.conectado[S] = true; pt.fase = F_EN_CURSO;
    pt.tKA = millis(); pt.fallos[0] = pt.fallos[1] = 0;
    Serial.printf("PUENTE CONECTADO: nodo %u (%s) <-> nodo %u (%s)\n",
                  pt.real[X], RED[X], pt.real[1 - X], RED[1 - X]);
  }
}

void deCentral(uint8_t S, const Paquete &p) {
  uint8_t v = p.destino;
  if (!esVirtual(S, v)) return;
  switch (p.tipo) {
    case ENTRANTE: entrante(S, v, p.dato); break;
    case CONECTAR: conectar(S, v, p.canal); break;
    case RESPUESTA:
      if (p.dato == R_TIMBRANDO || pt.fase == F_LIBRE || v != pt.virt[S]) break;
      Serial.printf("Respuesta de la central %s: codigo %u\n", RED[S], p.dato);
      terminar("respuesta de la central");
      break;
    case COLGAR:
      if (pt.fase != F_LIBRE && v == pt.virt[S]) terminar("la central cerro la llamada");
      break;
  }
}

void deNodo(uint8_t S, const Paquete &p) {
  if (pt.fase != F_EN_CURSO || p.origen != pt.real[S]) return;
  uint8_t O = 1 - S;
  switch (p.tipo) {
    case TEXTO: {
      Paquete q = armar(O, pt.virt[O], pt.real[O], TEXTO, 0, p.texto);
      transmitir(O, q);
      break;
    }
    case COLGAR:
      terminar("colgo el nodo", S);
      break;
  }
}

void voz(uint8_t S, uint8_t *b) {
  if (pt.fase != F_EN_CURSO || b[1] != pt.real[S]) return;
  uint8_t O = 1 - S;
  b[1] = pt.virt[O];               // el destino espera como origen al id virtual de su pareja
  txRaw(O, pt.real[O], b);
}

void alRecibir(uint8_t S, uint8_t *buf) {
  if (buf[0] == VOZ) { voz(S, buf); return; }
  Paquete p; memcpy(&p, buf, sizeof(p));
  p.texto[sizeof(p.texto) - 1] = '\0';
  if (p.origen == ID_CENTRAL) deCentral(S, p); else deNodo(S, p);
}

// ------------------------------------------------------------------------------------
void leerSerie() {
  static String linea = "";
  while (Serial.available()) {
    char c = Serial.read();
    if (c != '\n' && c != '\r') { if (linea.length() < 60) linea += c; continue; }
    linea.trim();
    if (linea == "/estado") {
      const char *n[] = {"libre", "pidiendo la llamada", "conectando", "en curso"};
      Serial.printf("Puente: %s", n[pt.fase]);
      if (pt.fase != F_LIBRE)
        Serial.printf(" | nodo %u (%s) <-> nodo %u (%s)", pt.real[pt.lx], RED[pt.lx], pt.real[1 - pt.lx], RED[1 - pt.lx]);
      Serial.println();
    }
    else if (linea == "/colgar") terminar("manual");
    else if (linea == "/info") {
      for (uint8_t S = 0; S < 2; S++) {
        Serial.printf("Red %s, control %u. Virtuales aqui:", RED[S], CONTROL[S]);
        for (uint8_t i = 0; i < NUM_NODOS[1 - S]; i++) Serial.printf(" %u", OFFSET_VIRTUAL + NODOS[1 - S][i]);
        Serial.println();
      }
    }
    linea = "";
  }
}

bool iniciarRadio(uint8_t S) {
  RF24 &r = *R[S];
  if (!r.begin(&SPI)) return false;
  r.setChannel(CONTROL[S]);
  r.setDataRate(RF24_1MBPS);
  r.setPALevel(RF24_PA_LOW);
  r.setPayloadSize(sizeof(Paquete));
  r.setAutoAck(true);
  r.enableDynamicAck();
  r.setRetries(5, 15);
  for (uint8_t i = 0; i < NUM_NODOS[1 - S]; i++) {     // un canal de recepcion por nodo virtual
    uint8_t d[5]; dir(S, OFFSET_VIRTUAL + NODOS[1 - S][i], d);
    r.openReadingPipe(i + 1, d);
  }
  r.startListening();
  canalAct[S] = CONTROL[S];
  return true;
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CSN_A);
  for (uint8_t S = 0; S < 2; S++) {
    if (!iniciarRadio(S)) {
      Serial.printf("ERROR: el nRF24 del lado %c no responde. Revisa cables y alimentacion.\n", S ? 'B' : 'A');
      while (true) delay(1000);
    }
  }
  pt = Puente{};
  tProxRegistro = millis();
  Serial.println("=== PUENTE DE REDES listo ===");
  for (uint8_t S = 0; S < 2; S++) {
    Serial.printf("Red %s (canal %u): el puente aparece como nodo(s)", RED[S], CONTROL[S]);
    for (uint8_t i = 0; i < NUM_NODOS[1 - S]; i++) Serial.printf(" %u", OFFSET_VIRTUAL + NODOS[1 - S][i]);
    Serial.println();
  }
}

void loop() {
  leerSerie();

  for (uint8_t S = 0; S < 2; S++) {
    uint8_t pipe;
    for (uint8_t n = 0; n < 8 && R[S]->available(&pipe); n++) {
      uint8_t buf[32];
      R[S]->read(buf, sizeof(buf));
      alRecibir(S, buf);
    }
  }

  unsigned long ahora = millis();

  if (pt.fase == F_LIBRE && (long)(ahora - tProxRegistro) >= 0) {
    registrar();
    tProxRegistro = ahora + REGISTRO_CADA_MS + random(500);
  }

  if ((pt.fase == F_PIDIENDO || pt.fase == F_CONECTANDO) && ahora - pt.desde > ESPERA_MAX_MS)
    terminar("tiempo de espera agotado");

  // Llamada en curso: avisos de presencia a los dos nodos y vigilancia por ACK
  if (pt.fase == F_EN_CURSO && ahora - pt.tKA >= KEEPALIVE_CADA_MS) {
    pt.tKA = ahora;
    for (uint8_t S = 0; S < 2 && pt.fase == F_EN_CURSO; S++) {
      bool ok = transmitir(S, armar(S, pt.virt[S], pt.real[S], KEEPALIVE, 0, nullptr));
      if (ok) pt.fallos[S] = 0;
      else if (++pt.fallos[S] >= FALLOS_MAX) terminar("un nodo dejo de responder", S);
    }
  }
}
