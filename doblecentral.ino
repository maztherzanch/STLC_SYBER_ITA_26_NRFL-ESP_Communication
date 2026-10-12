/*
  NODO TELEFONO - Topologia estrella con doble central redundante
  -------------------------------------------------------------------------------------
  Se carga este MISMO codigo en todos los telefonos; solo cambia MI_ID (1 a 254).
  Funciona tanto con la Central 1 (ID 0) como con la Central 2 (ID 100).
  Si una central se apaga, conmuta automaticamente a la otra.
*/
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// ================== CONFIGURA AQUI ==================
#define MI_ID 1          // <-- Cambiar en cada telefono: 1, 2, 3, 4...
// ====================================================

#if defined(CONFIG_IDF_TARGET_ESP32C3)
  // ---- ESP32-C3 SuperMini ----
  #define PIN_CE    3
  #define PIN_CSN   7
  #define PIN_SCK   4
  #define PIN_MISO  5
  #define PIN_MOSI  6
  #define PIN_LED   8
  #define PIN_BOOT  9
  #define PIN_MIC   1        // ADC1 (GPIO 0-4 son los analogicos del C3)
  #define LED_ON    LOW      // el LED de la placa enciende con LOW
  #define PLACA     "ESP32-C3 SuperMini"
#elif defined(CONFIG_IDF_TARGET_ESP32)
  // ---- ESP32 DevKit normal (ESP-WROOM-32) ----
  #define PIN_CE    2
  #define PIN_CSN   4
  #define PIN_SCK   18
  #define PIN_MISO  19
  #define PIN_MOSI  23
  #define PIN_LED   12
  #define PIN_BOOT  0
  #define PIN_MIC   34       // ADC1, solo entrada
  #define LED_ON    HIGH     // LED externo: enciende con HIGH
  #define PLACA     "ESP32 DevKit"
#else
  #error "Placa no soportada: elige ESP32 Dev Module o ESP32C3 Dev Module en Tools -> Board"
#endif
#define LED_OFF (LED_ON == HIGH ? LOW : HIGH)

// ======================= PROTOCOLO =======================
#define ID_CENTRAL_1   0
#define ID_CENTRAL_2   100
#define CANAL_CONTROL  80

enum Tipo : uint8_t {
  REGISTRO = 1, LLAMAR, CONTESTAR, RECHAZAR, LIBERAR,
  ENTRANTE, CONECTAR, RESPUESTA, TEXTO, KEEPALIVE, COLGAR,
  AVISO,         // central -> nodo : mensaje de la central (v2)
  VOZ            // nodo <-> nodo   : nivel de voz 0-255 en 'dato' (v3)
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

enum Estado { REPOSO, LLAMANDO, TIMBRANDO, EN_LLAMADA };

void direccionDe(uint8_t id, uint8_t *dir) {
  dir[0] = id; dir[1] = 'N'; dir[2] = 'O'; dir[3] = 'D'; dir[4] = 'E';
}

RF24 radio(PIN_CE, PIN_CSN);
uint8_t secuencia = 0;

Estado estado = REPOSO;
uint8_t pareja = 0;              // con quien hablo
uint8_t canalActual = CANAL_CONTROL;
unsigned long tEstado = 0;
unsigned long tUltimoEnvio = 0;
unsigned long tRegistro = 0;
uint8_t fallosSeguidos = 0;
bool centralPerdida = false;
uint8_t centralActiva = ID_CENTRAL_1;

// ---- Microfono ----
#define VENTANA_MS     50
int   micMin = 4095, micMax = 0;
unsigned long tVentana = 0;
int   picoAPico = 0;
int   ruidoBase = 10;
int   ganancia = 4;
uint8_t nivelMic = 0;
bool  ventanaNueva = false;
bool  modoMic = false;
uint8_t nivelRemoto = 0;
unsigned long tVozRemota = 0;
unsigned long tContacto = 0;
#define SIN_CONTACTO_MS 10000UL

#define REGISTRO_CADA_MS   5000UL
#define ESPERA_MAX_MS      25000UL

const char *nombreEstado[] = {"REPOSO", "LLAMANDO", "TIMBRANDO", "EN LLAMADA"};

// ------------------------------------------------------------------------------------
// Envio con redundancia automatica entre Central 1 y Central 2
bool enviar(uint8_t destino, uint8_t tipo, uint8_t dato, const char *texto) {
  Paquete p = {};
  p.tipo = tipo; p.origen = MI_ID; p.destino = destino;
  p.canal = canalActual; p.dato = dato; p.secuencia = secuencia++;
  if (texto) strncpy(p.texto, texto, sizeof(p.texto) - 1);
  
  uint8_t dir[5]; direccionDe(destino, dir);
  radio.stopListening();
  radio.openWritingPipe(dir);
  bool ok = radio.write(&p, sizeof(p));

  // Si intentaba hablar con la central y fallo, intenta con la otra central
  if (!ok && (destino == ID_CENTRAL_1 || destino == ID_CENTRAL_2)) {
    uint8_t idRespaldo = (destino == ID_CENTRAL_1) ? ID_CENTRAL_2 : ID_CENTRAL_1;
    p.destino = idRespaldo;
    direccionDe(idRespaldo, dir);
    radio.openWritingPipe(dir);
    ok = radio.write(&p, sizeof(p));
    if (ok) centralActiva = idRespaldo;
  }

  radio.startListening();
  tUltimoEnvio = millis();
  return ok;
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
  tContacto = millis();
  fallosSeguidos = 0;
  nivelRemoto = 0;
}

void ledBrillo(uint8_t v) {
  analogWrite(PIN_LED, LED_ON == HIGH ? v : 255 - v);
}

void leerMicrofono() {
  int v = analogRead(PIN_MIC);
  if (v < micMin) micMin = v;
  if (v > micMax) micMax = v;
  if (millis() - tVentana < VENTANA_MS) return;

  tVentana = millis();
  picoAPico = micMax - micMin;
  micMin = 4095; micMax = 0;

  int n = (picoAPico - ruidoBase) * ganancia;
  n = constrain(n, 0, 255);
  if (n > nivelMic) nivelMic = n;
  else nivelMic = (nivelMic * 2 + n) / 3;
  ventanaNueva = true;
}

void calibrarMicrofono() {
  Serial.println("Calibrando microfono: guarda silencio 1 segundo...");
  long suma = 0; int ventanas = 0;
  unsigned long t0 = millis();
  while (millis() - t0 < 1000) {
    int mn = 4095, mx = 0;
    unsigned long tw = millis();
    while (millis() - tw < VENTANA_MS) {
      int v = analogRead(PIN_MIC);
      if (v < mn) mn = v;
      if (v > mx) mx = v;
    }
    suma += mx - mn; ventanas++;
  }
  ruidoBase = suma / ventanas + 3;
  Serial.printf("Listo. Ruido de fondo = %d (escala 0-4095), ganancia = %d\n", ruidoBase, ganancia);
}

uint8_t gamma8(uint8_t v) { return (uint16_t)v * v / 255; }

void volverAReposo(bool avisarCentral) {
  if (canalActual != CANAL_CONTROL) cambiarCanal(CANAL_CONTROL);
  if (avisarCentral) enviar(centralActiva, LIBERAR, 0, nullptr);
  pareja = 0;
  cambiarEstado(REPOSO);
  enviar(centralActiva, REGISTRO, 0, nullptr);
  tRegistro = millis();
  Serial.println("En reposo (canal de control).");
}

// ------------------------------------------------------------------------------------
void procesarLinea(String l) {
  l.trim();
  if (l.length() == 0) return;

  if (l.startsWith("/llamar")) {
    if (estado != REPOSO) { Serial.println("Primero termina lo que estas haciendo (/colgar)."); return; }
    int destino = l.substring(7).toInt();
    if (destino < 1 || destino > 254 || destino == MI_ID) { Serial.println("Uso: /llamar ID"); return; }
    if (!enviar(centralActiva, LLAMAR, destino, nullptr)) { Serial.println("Ninguna central responde."); return; }
    pareja = destino;
    cambiarEstado(LLAMANDO);
    Serial.printf("Llamando al nodo %u...\n", destino);
  }
  else if (l == "/contestar") {
    if (estado != TIMBRANDO) { Serial.println("No hay llamada entrante."); return; }
    enviar(centralActiva, CONTESTAR, 0, nullptr);
    Serial.println("Contestando, esperando canal de la central...");
  }
  else if (l == "/rechazar") {
    if (estado != TIMBRANDO) { Serial.println("No hay llamada entrante."); return; }
    enviar(centralActiva, RECHAZAR, 0, nullptr);
    Serial.println("Llamada rechazada.");
    volverAReposo(false);
  }
  else if (l == "/colgar") {
    if (estado == EN_LLAMADA) {
      enviar(pareja, COLGAR, 0, nullptr);
      Serial.println("Colgaste.");
      volverAReposo(true);
    } else if (estado == LLAMANDO || estado == TIMBRANDO) {
      Serial.println("Llamada cancelada.");
      volverAReposo(true);
    } else Serial.println("No estas en llamada.");
  }
  else if (l == "/estado") {
    Serial.printf("Nodo %u | %s | canal %u (%u MHz)", MI_ID, nombreEstado[estado],
                  canalActual, 2400 + canalActual);
    if (pareja) Serial.printf(" | con nodo %u", pareja);
    Serial.printf(" | Central conectada: %u\n", centralActiva);
  }
  else if (l == "/mic") {
    modoMic = !modoMic;
    Serial.println(modoMic ? "Mostrando nivel del microfono (/mic para parar)" : "Parado.");
  }
  else if (l.startsWith("/ganancia")) {
    int g = l.substring(9).toInt();
    if (g < 1 || g > 20) { Serial.println("Uso: /ganancia 1..20"); return; }
    ganancia = g;
    Serial.printf("Ganancia = %d\n", ganancia);
  }
  else if (l == "/calibrar") calibrarMicrofono();
  else if (l.startsWith("/")) {
    Serial.println("Comandos: /llamar ID  /contestar  /rechazar  /colgar  /estado  /mic  /ganancia N  /calibrar");
  }
  else {
    if (estado != EN_LLAMADA) { Serial.println("No estas en llamada. Usa /llamar ID"); return; }
    String t = l.substring(0, 25);
    bool ok = enviar(pareja, TEXTO, 0, t.c_str());
    Serial.printf("[yo] %s %s\n", t.c_str(), ok ? "" : "(no llego)");
    if (l.length() > 25) Serial.println("  aviso: se corto a 25 caracteres");
  }
}

void procesarPaquete(Paquete &p) {
  p.texto[sizeof(p.texto) - 1] = '\0';

  // ---- Mensajes de cualquiera de las dos centrales ----
  if (p.origen == ID_CENTRAL_1 || p.origen == ID_CENTRAL_2) {
    centralActiva = p.origen;
    centralPerdida = false;
    switch (p.tipo) {
      case AVISO:
        Serial.printf("\n[CENTRAL %u] %s\n", p.origen, p.texto);
        break;

      case ENTRANTE:
        if (estado != REPOSO) break;
        pareja = p.dato;
        cambiarEstado(TIMBRANDO);
        Serial.printf("\n*** RING RING *** El nodo %u te llama. /contestar o /rechazar\n", pareja);
        break;

      case CONECTAR:
        if (estado != LLAMANDO && estado != TIMBRANDO) break;
        pareja = p.dato;
        cambiarCanal(p.canal);
        cambiarEstado(EN_LLAMADA);
        Serial.printf("CONECTADO con nodo %u en canal %u (%u MHz). Escribe para hablar, /colgar para terminar.\n",
                      pareja, p.canal, 2400 + p.canal);
        break;

      case RESPUESTA:
        switch (p.dato) {
          case R_TIMBRANDO:     Serial.printf("Timbrando en el nodo %u...\n", pareja); break;
          case R_OCUPADO:       Serial.println("El nodo esta OCUPADO.");        volverAReposo(false); break;
          case R_NO_DISPONIBLE: Serial.println("El nodo NO ESTA DISPONIBLE.");  volverAReposo(false); break;
          case R_RECHAZADO:     Serial.println("Llamada RECHAZADA.");           volverAReposo(false); break;
          case R_NO_CONTESTA:   Serial.println("El nodo NO CONTESTA.");         volverAReposo(false); break;
          case R_SIN_CANALES:   Serial.println("Central saturada: no hay canales libres."); volverAReposo(false); break;
          case R_CANCELADO:
            if (estado == TIMBRANDO) { Serial.println("La llamada se cancelo."); volverAReposo(false); }
            break;
        }
        break;
    }
    return;
  }

  // ---- Mensajes de la pareja ----
  if (estado != EN_LLAMADA || p.origen != pareja) return;
  fallosSeguidos = 0;
  tContacto = millis();
  switch (p.tipo) {
    case VOZ:    nivelRemoto = p.dato; tVozRemota = millis(); break;
    case TEXTO:  Serial.printf("[nodo %u] %s\n", p.origen, p.texto); break;
    case COLGAR: Serial.printf("El nodo %u colgo.\n", p.origen); volverAReposo(true); break;
    case KEEPALIVE: break;
  }
}

void tareasPeriodicas() {
  unsigned long ahora = millis();

  if (estado == REPOSO && ahora - tRegistro > REGISTRO_CADA_MS) {
    tRegistro = ahora;
    bool ok = enviar(centralActiva, REGISTRO, 0, nullptr);
    if (!ok && !centralPerdida) { Serial.println("Aviso: ninguna central responde."); centralPerdida = true; }
    if (ok && centralPerdida)   { Serial.printf("Central %u encontrada.\n", centralActiva); centralPerdida = false; }
  }

  if ((estado == LLAMANDO || estado == TIMBRANDO) && ahora - tEstado > ESPERA_MAX_MS) {
    Serial.println("Tiempo de espera agotado.");
    volverAReposo(true);
  }

  if (estado == EN_LLAMADA && ventanaNueva) {
    if (enviar(pareja, VOZ, nivelMic, nullptr)) tContacto = ahora;
    if (ahora - tContacto > SIN_CONTACTO_MS) {
      Serial.println("Se perdio la conexion con la pareja. Llamada terminada.");
      volverAReposo(true);
    }
  }
  if (ahora - tVozRemota > 300) nivelRemoto = 0;

  static unsigned long tBarra = 0;
  if (modoMic && ventanaNueva && ahora - tBarra >= 100) {
    tBarra = ahora;
    Serial.printf("mic: amplitud %4d  nivel %3u |", picoAPico, nivelMic);
    for (int i = 0; i < nivelMic / 8; i++) Serial.print('#');
    Serial.println();
  }
  ventanaNueva = false;

  uint8_t brillo = 0;
  if (estado == TIMBRANDO)  brillo = ((ahora / 150) % 2) ? 255 : 0;
  if (estado == LLAMANDO)   brillo = ((ahora / 600) % 2) ? 255 : 0;
  if (estado == EN_LLAMADA) brillo = max((uint8_t)6, gamma8(nivelRemoto));
  if (estado == REPOSO && modoMic) brillo = gamma8(nivelMic);
  ledBrillo(brillo);
}

// ------------------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(1500);
  pinMode(PIN_LED, OUTPUT);
  ledBrillo(0);
  pinMode(PIN_BOOT, INPUT_PULLUP);
  pinMode(PIN_MIC, INPUT);
  analogReadResolution(12);

  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CSN);
  if (!radio.begin(&SPI)) {
    Serial.println("ERROR: el nRF24L01 no responde. Revisa cables y alimentacion.");
    while (true) { ledBrillo(255); delay(100); ledBrillo(0); delay(100); }
  }
  radio.setChannel(CANAL_CONTROL);
  radio.setDataRate(RF24_1MBPS);
  radio.setPALevel(RF24_PA_LOW);
  radio.setPayloadSize(sizeof(Paquete));
  radio.setAutoAck(true);
  radio.setRetries(3 + (MI_ID % 10), 15);
  uint8_t dir[5]; direccionDe(MI_ID, dir);
  radio.openReadingPipe(1, dir);
  radio.startListening();

  Serial.printf("\n=== Nodo telefono %u listo (%s) ===\n", MI_ID, PLACA);
  Serial.println("Comandos: /llamar ID  /contestar  /rechazar  /colgar  /estado  /mic  /ganancia N  /calibrar");
  calibrarMicrofono();
  tRegistro = millis() - REGISTRO_CADA_MS;
}

void loop() {
  static String linea = "";
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') { procesarLinea(linea); linea = ""; }
    else if (linea.length() < 100) linea += c;
  }

  static unsigned long tBoton = 0;
  if (digitalRead(PIN_BOOT) == LOW && millis() - tBoton > 600) {
    tBoton = millis();
    if (estado == TIMBRANDO) procesarLinea("/contestar");
    else if (estado == EN_LLAMADA) procesarLinea("/colgar");
  }

  if (radio.available()) {
    Paquete p;
    radio.read(&p, sizeof(p));
    procesarPaquete(p);
  }

  leerMicrofono();
  tareasPeriodicas();
}