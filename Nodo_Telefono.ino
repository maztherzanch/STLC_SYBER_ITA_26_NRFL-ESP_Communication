/*
  NODO TELEFONO - Topologia estrella con central (nRF24L01+ PA+LNA + ESP32)
  -------------------------------------------------------------------------------------
  Se carga el MISMO codigo en todos los nodos; solo cambia MI_ID (1 a 254).
  Necesita que la CENTRAL (Central_Estrella.ino, ID 0) este encendida.

  Comandos en el Monitor Serial:
     /llamar 2    -> pide a la central llamar al nodo 2
     /contestar   -> contesta una llamada entrante
     /rechazar    -> rechaza una llamada entrante
     /colgar      -> termina la llamada (o cancela si aun esta timbrando)
     /estado      -> muestra en que estado y canal esta el nodo
     texto normal -> durante una llamada, se lo manda a la otra persona

  Boton BOOT: contesta si esta timbrando, cuelga si esta en llamada.
  LED: parpadea rapido = timbrando, lento = llamando, fijo = en llamada.

  FUNCIONA EN DOS PLACAS: el codigo detecta solo cual elegiste en
  Tools -> Board y usa los pines correctos. Solo cablea segun tu placa:

  a) ESP32 DevKit normal (ESP-WROOM-32) -> placa "ESP32 Dev Module"
     Mismo cableado que el circuito del profe:
     VCC->3V3  GND->GND  CE->GPIO2  CSN->GPIO4
     SCK->GPIO18  MISO->GPIO19  MOSI->GPIO23
     LED externo en GPIO12 (con resistencia de 330 ohm a GND)
     Boton BOOT de la placa = GPIO0

  b) ESP32-C3 SuperMini -> placa "ESP32C3 Dev Module", "USB CDC On Boot: Enabled"
     VCC->3V3  GND->GND  CE->GPIO3  CSN->GPIO7
     SCK->GPIO4  MOSI->GPIO6  MISO->GPIO5
     LED de la placa = GPIO8, boton BOOT = GPIO9

  En ambos: capacitor 10-100uF entre VCC y GND del nRF.
  Monitor Serial a 115200 con "Nueva linea".
*/
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// ================== CONFIGURA AQUI ==================
#define MI_ID 1          // <-- distinto en cada nodo: 1, 2, 3 ...
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
  #define LED_ON    LOW      // el LED de la placa enciende con LOW
  #define PLACA     "ESP32-C3 SuperMini"
#elif defined(CONFIG_IDF_TARGET_ESP32)
  // ---- ESP32 DevKit normal (ESP-WROOM-32), cableado del profe ----
  // OJO: en este ESP32 los GPIO 6 a 11 son de la memoria flash interna;
  // si se usan, la placa se reinicia sin parar.
  #define PIN_CE    2
  #define PIN_CSN   4
  #define PIN_SCK   18
  #define PIN_MISO  19
  #define PIN_MOSI  23
  #define PIN_LED   12
  #define PIN_BOOT  0
  #define LED_ON    HIGH     // LED externo: enciende con HIGH
  #define PLACA     "ESP32 DevKit"
#else
  #error "Placa no soportada: elige ESP32 Dev Module o ESP32C3 Dev Module en Tools -> Board"
#endif
#define LED_OFF (LED_ON == HIGH ? LOW : HIGH)

// ======================= PROTOCOLO (igual en central y nodos) =======================
#define ID_CENTRAL     0
#define CANAL_CONTROL  80

enum Tipo : uint8_t {
  REGISTRO = 1, LLAMAR, CONTESTAR, RECHAZAR, LIBERAR,
  ENTRANTE, CONECTAR, RESPUESTA, TEXTO, KEEPALIVE, COLGAR
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
// Estados del telefono. Va ANTES de la primera funcion: el IDE de Arduino
// genera prototipos automaticos arriba de ella y necesita conocer este tipo.
enum Estado { REPOSO, LLAMANDO, TIMBRANDO, EN_LLAMADA };

void direccionDe(uint8_t id, uint8_t *dir) {
  dir[0] = id; dir[1] = 'N'; dir[2] = 'O'; dir[3] = 'D'; dir[4] = 'E';
}
// ===================================================================================

RF24 radio(PIN_CE, PIN_CSN);
uint8_t secuencia = 0;

Estado estado = REPOSO;
uint8_t pareja = 0;              // con quien hablo
uint8_t canalActual = CANAL_CONTROL;
unsigned long tEstado = 0;       // cuando entre al estado actual
unsigned long tUltimoEnvio = 0;
unsigned long tRegistro = 0;
uint8_t fallosSeguidos = 0;
bool centralPerdida = false;

#define REGISTRO_CADA_MS   5000UL
#define ESPERA_MAX_MS      25000UL  // tiempo maximo llamando / timbrando
#define KEEPALIVE_CADA_MS  2000UL
#define FALLOS_PARA_CORTAR 5        // ~10 s sin contacto con la pareja -> se corta

const char *nombreEstado[] = {"REPOSO", "LLAMANDO", "TIMBRANDO", "EN LLAMADA"};

// ------------------------------------------------------------------------------------
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

void cambiarCanal(uint8_t c) {
  radio.stopListening();
  radio.setChannel(c);
  radio.startListening();
  canalActual = c;
}

void cambiarEstado(Estado e) {
  estado = e;
  tEstado = millis();
  fallosSeguidos = 0;
}

// Regresa al canal de control y avisa a la central que el nodo quedo libre
void volverAReposo(bool avisarCentral) {
  if (canalActual != CANAL_CONTROL) cambiarCanal(CANAL_CONTROL);
  if (avisarCentral) enviar(ID_CENTRAL, LIBERAR, 0, nullptr);
  pareja = 0;
  cambiarEstado(REPOSO);
  enviar(ID_CENTRAL, REGISTRO, 0, nullptr);
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
    if (!enviar(ID_CENTRAL, LLAMAR, destino, nullptr)) { Serial.println("La central no responde."); return; }
    pareja = destino;
    cambiarEstado(LLAMANDO);
    Serial.printf("Llamando al nodo %u...\n", destino);
  }
  else if (l == "/contestar") {
    if (estado != TIMBRANDO) { Serial.println("No hay llamada entrante."); return; }
    enviar(ID_CENTRAL, CONTESTAR, 0, nullptr);
    Serial.println("Contestando, esperando canal de la central...");
  }
  else if (l == "/rechazar") {
    if (estado != TIMBRANDO) { Serial.println("No hay llamada entrante."); return; }
    enviar(ID_CENTRAL, RECHAZAR, 0, nullptr);
    Serial.println("Llamada rechazada.");
    volverAReposo(false);
  }
  else if (l == "/colgar") {
    if (estado == EN_LLAMADA) {
      enviar(pareja, COLGAR, 0, nullptr);                 // avisar a la otra persona
      Serial.println("Colgaste.");
      volverAReposo(true);
    } else if (estado == LLAMANDO || estado == TIMBRANDO) {
      Serial.println("Llamada cancelada.");
      volverAReposo(true);                    // la central avisa al otro
    } else Serial.println("No estas en llamada.");
  }
  else if (l == "/estado") {
    Serial.printf("Nodo %u | %s | canal %u (%u MHz)", MI_ID, nombreEstado[estado],
                  canalActual, 2400 + canalActual);
    if (pareja) Serial.printf(" | con nodo %u", pareja);
    Serial.println();
  }
  else if (l.startsWith("/")) {
    Serial.println("Comandos: /llamar ID  /contestar  /rechazar  /colgar  /estado");
  }
  else {                                       // texto normal
    if (estado != EN_LLAMADA) { Serial.println("No estas en llamada. Usa /llamar ID"); return; }
    String t = l.substring(0, 25);
    bool ok = enviar(pareja, TEXTO, 0, t.c_str());
    Serial.printf("[yo] %s %s\n", t.c_str(), ok ? "" : "(no llego)");
    if (l.length() > 25) Serial.println("  aviso: se corto a 25 caracteres");
  }
}

void procesarPaquete(Paquete &p) {
  p.texto[sizeof(p.texto) - 1] = '\0';

  // ---- Mensajes de la central (canal de control) ----
  if (p.origen == ID_CENTRAL) {
    centralPerdida = false;
    switch (p.tipo) {
      case ENTRANTE:
        if (estado != REPOSO) break;
        pareja = p.dato;
        cambiarEstado(TIMBRANDO);
        Serial.printf("\n*** RING RING *** El nodo %u te llama. /contestar o /rechazar\n", pareja);
        break;

      case CONECTAR:
        if (estado != LLAMANDO && estado != TIMBRANDO) break;
        pareja = p.dato;
        cambiarCanal(p.canal);                // <-- nos vamos a nuestra propia linea
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

  // ---- Mensajes de la pareja (en nuestro canal de voz) ----
  if (estado != EN_LLAMADA || p.origen != pareja) return;
  fallosSeguidos = 0;
  switch (p.tipo) {
    case TEXTO:  Serial.printf("[nodo %u] %s\n", p.origen, p.texto); break;
    case COLGAR: Serial.printf("El nodo %u colgo.\n", p.origen); volverAReposo(true); break;
    case KEEPALIVE: break;
  }
}

// Tareas periodicas segun el estado
void tareasPeriodicas() {
  unsigned long ahora = millis();

  if (estado == REPOSO && ahora - tRegistro > REGISTRO_CADA_MS) {
    tRegistro = ahora;
    bool ok = enviar(ID_CENTRAL, REGISTRO, 0, nullptr);
    if (!ok && !centralPerdida) { Serial.println("Aviso: la central no responde."); centralPerdida = true; }
    if (ok && centralPerdida)   { Serial.println("Central encontrada.");            centralPerdida = false; }
  }

  if ((estado == LLAMANDO || estado == TIMBRANDO) && ahora - tEstado > ESPERA_MAX_MS) {
    Serial.println("Tiempo de espera agotado.");
    volverAReposo(true);
  }

  // En llamada: comprobar que la pareja sigue ahi
  if (estado == EN_LLAMADA && ahora - tUltimoEnvio > KEEPALIVE_CADA_MS) {
    if (enviar(pareja, KEEPALIVE, 0, nullptr)) fallosSeguidos = 0;
    else if (++fallosSeguidos >= FALLOS_PARA_CORTAR) {
      Serial.println("Se perdio la conexion con la pareja. Llamada terminada.");
      volverAReposo(true);
    }
  }

  // LED: rapido = timbrando, lento = llamando, fijo = en llamada
  bool led = false;
  if (estado == TIMBRANDO)  led = (ahora / 150) % 2;
  if (estado == LLAMANDO)   led = (ahora / 600) % 2;
  if (estado == EN_LLAMADA) led = true;
  digitalWrite(PIN_LED, led ? LED_ON : LED_OFF);
}

// ------------------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(1500);
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LED_OFF);
  pinMode(PIN_BOOT, INPUT_PULLUP);

  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CSN);
  if (!radio.begin(&SPI)) {
    Serial.println("ERROR: el nRF24L01 no responde. Revisa cables y alimentacion.");
    while (true) { digitalWrite(PIN_LED, !digitalRead(PIN_LED)); delay(100); }
  }
  radio.setChannel(CANAL_CONTROL);
  radio.setDataRate(RF24_1MBPS);
  radio.setPALevel(RF24_PA_LOW);
  radio.setPayloadSize(sizeof(Paquete));
  radio.setAutoAck(true);
  radio.setRetries(3 + (MI_ID % 10), 15);    // reintentos escalonados por nodo
  uint8_t dir[5]; direccionDe(MI_ID, dir);
  radio.openReadingPipe(1, dir);
  radio.startListening();

  Serial.printf("\n=== Nodo telefono %u listo (%s) ===\n", MI_ID, PLACA);
  Serial.println("Comandos: /llamar ID  /contestar  /rechazar  /colgar  /estado");
  tRegistro = millis() - REGISTRO_CADA_MS;    // registrarse de inmediato
}

void loop() {
  static String linea = "";
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') { procesarLinea(linea); linea = ""; }
    else if (linea.length() < 100) linea += c;
  }

  // Boton BOOT: contestar o colgar
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

  tareasPeriodicas();
}
