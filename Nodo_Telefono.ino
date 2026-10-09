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

     /mic         -> muestra en vivo el nivel de tu microfono (para probarlo)
     /ganancia N  -> sensibilidad del microfono (1 a 20, por defecto 4)
     /calibrar    -> vuelve a medir el ruido de fondo (guarda silencio 1 s)

  Mensajes de la central: aparecen como [CENTRAL] ... en cualquier momento,
  incluso a media llamada (v2).

  MICROFONO -> LED (v3): durante una llamada, cada telefono mide que tan fuerte
  le hablan a SU microfono y le manda ese nivel a la otra persona 20 veces por
  segundo. El LED de la otra persona brilla mas o menos segun tu voz.
  (Modulo de sonido KY-037/KY-038: se usa la salida analogica AO.)

  Boton BOOT: contesta si esta timbrando, cuelga si esta en llamada.
  LED: parpadea rapido = timbrando, lento = llamando,
       en llamada = brilla con la voz de la otra persona.

  FUNCIONA EN DOS PLACAS: el codigo detecta solo cual elegiste en
  Tools -> Board y usa los pines correctos. Solo cablea segun tu placa:

  a) ESP32 DevKit normal (ESP-WROOM-32) -> placa "ESP32 Dev Module"
     Mismo cableado que el circuito del profe:
     VCC->3V3  GND->GND  CE->GPIO2  CSN->GPIO4
     SCK->GPIO18  MISO->GPIO19  MOSI->GPIO23
     LED externo en GPIO12 (con resistencia de 330 ohm a GND)
     Boton BOOT de la placa = GPIO0
     Microfono: AO->GPIO34  +->3V3  GND->GND  (DO sin conectar)

  b) ESP32-C3 SuperMini -> placa "ESP32C3 Dev Module", "USB CDC On Boot: Enabled"
     VCC->3V3  GND->GND  CE->GPIO3  CSN->GPIO7
     SCK->GPIO4  MOSI->GPIO6  MISO->GPIO5
     LED de la placa = GPIO8, boton BOOT = GPIO9
     Microfono: AO->GPIO1  +->3V3  GND->GND  (DO sin conectar)

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
  #define PIN_MIC   1        // ADC1 (GPIO 0-4 son los analogicos del C3)
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
  #define PIN_MIC   34       // ADC1, solo entrada: ideal para el microfono
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

// ---- Microfono ----
#define VENTANA_MS     50        // se mide el volumen cada 50 ms (20 veces por segundo)
int   micMin = 4095, micMax = 0; // minimo y maximo dentro de la ventana actual
unsigned long tVentana = 0;
int   picoAPico = 0;             // amplitud de la ultima ventana (0-4095)
int   ruidoBase = 10;            // amplitud del silencio (se calibra al encender)
int   ganancia = 4;              // sensibilidad: mas alto = el LED reacciona a voz mas baja
uint8_t nivelMic = 0;            // MI volumen, 0-255 (con envolvente)
bool  ventanaNueva = false;
bool  modoMic = false;           // /mic: imprimir el nivel en vivo
uint8_t nivelRemoto = 0;         // volumen de la OTRA persona, 0-255
unsigned long tVozRemota = 0;
unsigned long tContacto = 0;     // ultima vez que supimos de la pareja
#define SIN_CONTACTO_MS 10000UL  // 10 s sin saber de la pareja -> se corta

#define REGISTRO_CADA_MS   5000UL
#define ESPERA_MAX_MS      25000UL  // tiempo maximo llamando / timbrando

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
  tContacto = millis();
  fallosSeguidos = 0;
  nivelRemoto = 0;
}

// ---- LED con brillo (PWM). 0 = apagado, 255 = maximo ----
void ledBrillo(uint8_t v) {
  analogWrite(PIN_LED, LED_ON == HIGH ? v : 255 - v);
}

// ---- Microfono ----
// Se llama en cada vuelta del loop: toma una muestra y, cada 50 ms,
// calcula el volumen como la diferencia entre el maximo y el minimo (pico a pico).
void leerMicrofono() {
  int v = analogRead(PIN_MIC);
  if (v < micMin) micMin = v;
  if (v > micMax) micMax = v;
  if (millis() - tVentana < VENTANA_MS) return;

  tVentana = millis();
  picoAPico = micMax - micMin;
  micMin = 4095; micMax = 0;

  int n = (picoAPico - ruidoBase) * ganancia;      // quitar el ruido de fondo y amplificar
  n = constrain(n, 0, 255);
  if (n > nivelMic) nivelMic = n;                  // sube rapido...
  else nivelMic = (nivelMic * 2 + n) / 3;          // ...y baja suave (se ve mas natural)
  ventanaNueva = true;
}

// Mide 1 segundo de silencio para saber cuanto "ruido" tiene el microfono solo.
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

// Brillo percibido: el ojo es mas sensible a cambios en niveles bajos (correccion gamma)
uint8_t gamma8(uint8_t v) { return (uint16_t)v * v / 255; }

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
  else if (l == "/mic") {
    modoMic = !modoMic;
    Serial.println(modoMic ? "Mostrando nivel del microfono (escribe /mic otra vez para parar)"
                           : "Listo, ya no se muestra el microfono.");
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
      case AVISO:      // mensaje de la central: se muestra en cualquier estado
        Serial.printf("\n[CENTRAL] %s\n", p.texto);
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
  tContacto = millis();
  switch (p.tipo) {
    case VOZ:    nivelRemoto = p.dato; tVozRemota = millis(); break;
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

  // En llamada: cada 50 ms mandar MI nivel de voz a la pareja.
  // Ese mismo envio sirve para comprobar que la pareja sigue ahi (ACK).
  if (estado == EN_LLAMADA && ventanaNueva) {
    if (enviar(pareja, VOZ, nivelMic, nullptr)) tContacto = ahora;
    if (ahora - tContacto > SIN_CONTACTO_MS) {
      Serial.println("Se perdio la conexion con la pareja. Llamada terminada.");
      volverAReposo(true);
    }
  }
  if (ahora - tVozRemota > 300) nivelRemoto = 0;   // si deja de llegar voz, apagar

  // /mic: barra con el nivel del microfono (cada 100 ms)
  static unsigned long tBarra = 0;
  if (modoMic && ventanaNueva && ahora - tBarra >= 100) {
    tBarra = ahora;
    Serial.printf("mic: amplitud %4d  nivel %3u |", picoAPico, nivelMic);
    for (int i = 0; i < nivelMic / 8; i++) Serial.print('#');
    Serial.println();
  }
  ventanaNueva = false;

  // LED
  uint8_t brillo = 0;
  if (estado == TIMBRANDO)  brillo = ((ahora / 150) % 2) ? 255 : 0;
  if (estado == LLAMANDO)   brillo = ((ahora / 600) % 2) ? 255 : 0;
  if (estado == EN_LLAMADA) brillo = max((uint8_t)6, gamma8(nivelRemoto));  // tenue = en llamada
  if (estado == REPOSO && modoMic) brillo = gamma8(nivelMic);              // prueba local
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
  analogReadResolution(12);                   // 0-4095

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
  radio.setRetries(3 + (MI_ID % 10), 15);    // reintentos escalonados por nodo
  uint8_t dir[5]; direccionDe(MI_ID, dir);
  radio.openReadingPipe(1, dir);
  radio.startListening();

  Serial.printf("\n=== Nodo telefono %u listo (%s) ===\n", MI_ID, PLACA);
  Serial.println("Comandos: /llamar ID  /contestar  /rechazar  /colgar  /estado  /mic  /ganancia N  /calibrar");
  calibrarMicrofono();
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

  leerMicrofono();
  tareasPeriodicas();
}
