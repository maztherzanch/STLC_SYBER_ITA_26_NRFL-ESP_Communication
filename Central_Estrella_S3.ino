/*
  CENTRAL TELEFONICA - Topologia estrella con nRF24L01+ PA+LNA + ESP32-S3 SuperMini
  -------------------------------------------------------------------------------
  Funciona como una central telefonica:
   1. Todos los nodos se registran con la central en el CANAL DE CONTROL.
   2. Un nodo pide "llamar al nodo X". La central revisa que X exista y este libre.
   3. La central hace timbrar a X. Si contesta, les asigna un CANAL LIBRE
      y los manda a los dos a ese canal ("su propia linea").
   4. Los dos nodos hablan DIRECTO en su canal; la central ya no interviene
      y queda libre para atender otras llamadas en el canal de control.
   5. Al colgar, los nodos regresan al canal de control y la central libera el canal.

  Comandos en el Monitor Serial de la central:
     /tabla    -> nodos registrados y su estado
     /canales  -> canales de voz y que llamada ocupa cada uno

  VERSION PARA ESP32-S3 SuperMini (los telefonos siguen con ESP32-C3;
  el protocolo de radio es el mismo, asi que se entienden sin problema).

  Conexiones nRF24L01+ PA+LNA -> ESP32-S3 SuperMini
     VCC  -> 3V3 (+capacitor 10-100uF)
     GND  -> GND
     CE   -> GPIO9
     CSN  -> GPIO10
     MOSI -> GPIO11
     SCK  -> GPIO12
     MISO -> GPIO13
     IRQ  -> sin conectar
  (Son los pines SPI de fabrica del S3 y estan todos juntos en el mismo
   lado de la placa, junto a 3V3 y GND.)

  LED: el S3 SuperMini trae un LED RGB (WS2812) en GPIO48:
     azul tenue = central lista, verde = hay llamadas en curso, rojo parpadeando = error.

  Arduino IDE: placa "ESP32S3 Dev Module", "USB CDC On Boot: Enabled", 115200 baud.
*/
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define PIN_CE    9
#define PIN_CSN   10
#define PIN_MOSI  11
#define PIN_SCK   12
#define PIN_MISO  13
#define PIN_RGB   48     // LED RGB WS2812 de la placa

// Enciende el LED RGB de la placa con el color indicado (0-255 cada uno)
void ledRGB(uint8_t r, uint8_t g, uint8_t b) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  rgbLedWrite(PIN_RGB, r, g, b);
#else
  neopixelWrite(PIN_RGB, r, g, b);
#endif
}

// ======================= PROTOCOLO (igual en central y nodos) =======================
#define ID_CENTRAL     0
#define CANAL_CONTROL  80        // 2480 MHz: canal donde todos esperan y piden llamadas
// Canales de voz que la central puede asignar (uno por llamada simultanea).
// Separados 2 MHz para no interferirse. Todos dentro de la banda ISM (0..83).
const uint8_t CANALES_VOZ[] = {70, 72, 74, 76, 78, 82};
const uint8_t NUM_CANALES = sizeof(CANALES_VOZ);

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
  COLGAR         // nodo <-> nodo   : termino la llamada
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

void direccionDe(uint8_t id, uint8_t *dir) {   // direccion de 5 bytes de cada nodo
  dir[0] = id; dir[1] = 'N'; dir[2] = 'O'; dir[3] = 'D'; dir[4] = 'E';
}
// ===================================================================================

RF24 radio(PIN_CE, PIN_CSN);
uint8_t secuencia = 0;

// ---- Tabla de nodos ----
#define MAX_NODOS 16
#define NODO_CADUCA_MS 15000UL       // si no se registra en 15 s, se da por desconectado
struct Nodo { uint8_t id; unsigned long visto; };
Nodo nodos[MAX_NODOS];
uint8_t numNodos = 0;

// ---- Tabla de llamadas (una por canal de voz) ----
#define TIMBRE_MAX_MS 20000UL        // 20 s timbrando sin contestar -> "no contesta"
enum EstadoLlamada : uint8_t { LIBRE = 0, TIMBRANDO, EN_CURSO };
struct Llamada {
  EstadoLlamada estado;
  uint8_t origen, destino;           // quien llama y a quien
  unsigned long desde;
};
Llamada llamadas[NUM_CANALES];

// ------------------------------------------------------------------------------------
bool enviar(uint8_t destino, uint8_t tipo, uint8_t canal, uint8_t dato) {
  Paquete p = {};
  p.tipo = tipo; p.origen = ID_CENTRAL; p.destino = destino;
  p.canal = canal; p.dato = dato; p.secuencia = secuencia++;
  uint8_t dir[5]; direccionDe(destino, dir);
  radio.stopListening();
  radio.openWritingPipe(dir);
  bool ok = radio.write(&p, sizeof(p));
  radio.startListening();
  return ok;
}

const char *nombreCodigo(uint8_t c) {
  switch (c) {
    case R_TIMBRANDO: return "timbrando"; case R_OCUPADO: return "ocupado";
    case R_NO_DISPONIBLE: return "no disponible"; case R_RECHAZADO: return "rechazada";
    case R_NO_CONTESTA: return "no contesta"; case R_SIN_CANALES: return "sin canales libres";
    case R_CANCELADO: return "cancelada"; default: return "?";
  }
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
int llamadaDe(uint8_t id) {           // indice de la llamada donde participa 'id', o -1
  for (int i = 0; i < NUM_CANALES; i++)
    if (llamadas[i].estado != LIBRE &&
        (llamadas[i].origen == id || llamadas[i].destino == id)) return i;
  return -1;
}
int canalLibre() {
  for (int i = 0; i < NUM_CANALES; i++) if (llamadas[i].estado == LIBRE) return i;
  return -1;
}
void liberar(int i, const char *motivo) {
  Serial.printf("Canal %u liberado (%u <-> %u): %s\n", CANALES_VOZ[i],
                llamadas[i].origen, llamadas[i].destino, motivo);
  llamadas[i].estado = LIBRE;
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
  int i = canalLibre();
  if (i < 0) { enviar(a, RESPUESTA, 0, R_SIN_CANALES); return; }

  // Reservar canal y hacer timbrar al destino
  llamadas[i] = Llamada{TIMBRANDO, a, b, millis()};
  if (!enviar(b, ENTRANTE, 0, a)) {
    liberar(i, "el destino no respondio");
    enviar(a, RESPUESTA, 0, R_NO_DISPONIBLE);
    return;
  }
  enviar(a, RESPUESTA, 0, R_TIMBRANDO);
  Serial.printf("  Timbrando en nodo %u (canal %u reservado)\n", b, CANALES_VOZ[i]);
}

void atenderContestar(uint8_t b) {
  int i = llamadaDe(b);
  if (i < 0 || llamadas[i].estado != TIMBRANDO || llamadas[i].destino != b) return;
  uint8_t a = llamadas[i].origen, canal = CANALES_VOZ[i];
  llamadas[i].estado = EN_CURSO;
  llamadas[i].desde = millis();
  // Mandar a los dos a su propio canal
  enviar(b, CONECTAR, canal, a);
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
  if (llamadas[i].estado == TIMBRANDO) {     // colgo antes de que contestaran
    uint8_t otro = (llamadas[i].origen == x) ? llamadas[i].destino : llamadas[i].origen;
    enviar(otro, RESPUESTA, 0, R_CANCELADO);
    liberar(i, "cancelada antes de contestar");
  } else {
    liberar(i, "colgaron");
  }
}

void revisarTiempos() {
  for (int i = 0; i < NUM_CANALES; i++) {
    if (llamadas[i].estado == TIMBRANDO && millis() - llamadas[i].desde > TIMBRE_MAX_MS) {
      enviar(llamadas[i].origen, RESPUESTA, 0, R_NO_CONTESTA);
      enviar(llamadas[i].destino, RESPUESTA, 0, R_CANCELADO);
      liberar(i, "no contesto");
    }
  }
  // LED encendido mientras haya alguna llamada en curso
  bool alguna = false;
  for (int i = 0; i < NUM_CANALES; i++) if (llamadas[i].estado == EN_CURSO) alguna = true;
  static int ledAnterior = -1;            // solo actualizar el LED cuando cambia
  if (alguna != ledAnterior) {
    ledAnterior = alguna;
    if (alguna) ledRGB(0, 40, 0);         // verde: llamadas en curso
    else        ledRGB(0, 0, 10);         // azul tenue: central lista
  }
}

// ------------------------------------------------------------------------------------
void mostrarTabla() {
  Serial.println("--- Nodos ---");
  if (numNodos == 0) Serial.println("  (ninguno)");
  for (int i = 0; i < numNodos; i++) {
    uint8_t id = nodos[i].id;
    int l = llamadaDe(id);
    const char *estado = !nodoEnLinea(id) && l < 0 ? "desconectado"
                       : l < 0 ? "libre"
                       : llamadas[l].estado == TIMBRANDO ? "timbrando" : "en llamada";
    Serial.printf("  Nodo %3u : %s\n", id, estado);
  }
}
void mostrarCanales() {
  Serial.printf("--- Canal de control: %u (%u MHz) ---\n", CANAL_CONTROL, 2400 + CANAL_CONTROL);
  for (int i = 0; i < NUM_CANALES; i++) {
    Serial.printf("  Canal %3u (%u MHz): ", CANALES_VOZ[i], 2400 + CANALES_VOZ[i]);
    if (llamadas[i].estado == LIBRE) Serial.println("libre");
    else Serial.printf("%s  %u -> %u  (%lu s)\n",
                       llamadas[i].estado == TIMBRANDO ? "timbrando" : "EN CURSO",
                       llamadas[i].origen, llamadas[i].destino,
                       (millis() - llamadas[i].desde) / 1000);
  }
}

// ------------------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(1500);
  ledRGB(0, 0, 0);

  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CSN);
  if (!radio.begin(&SPI)) {
    Serial.println("ERROR: el nRF24L01 no responde. Revisa cables y alimentacion.");
    while (true) { ledRGB(40, 0, 0); delay(150); ledRGB(0, 0, 0); delay(150); }
  }
  radio.setChannel(CANAL_CONTROL);
  radio.setDataRate(RF24_1MBPS);
  radio.setPALevel(RF24_PA_LOW);
  radio.setPayloadSize(sizeof(Paquete));
  radio.setAutoAck(true);
  radio.setRetries(5, 15);
  uint8_t dir[5]; direccionDe(ID_CENTRAL, dir);
  radio.openReadingPipe(1, dir);
  radio.startListening();

  Serial.println("\n=== CENTRAL TELEFONICA lista ===");
  mostrarCanales();
  Serial.println("Comandos: /tabla  /canales");
}

void loop() {
  // Comandos del Monitor Serial
  if (Serial.available()) {
    String l = Serial.readStringUntil('\n'); l.trim();
    if (l == "/tabla") mostrarTabla();
    else if (l == "/canales") mostrarCanales();
    else if (l.length()) Serial.println("Comandos: /tabla  /canales");
  }

  // Mensajes de los nodos
  if (radio.available()) {
    Paquete p;
    radio.read(&p, sizeof(p));
    registrarNodo(p.origen);            // cualquier mensaje cuenta como "sigue vivo"
    switch (p.tipo) {
      case REGISTRO: {
        // Si un nodo dice estar libre pero la central lo tiene en una llamada
        // en curso desde hace rato, es que se reinicio: liberar su canal.
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

  revisarTiempos();
}
