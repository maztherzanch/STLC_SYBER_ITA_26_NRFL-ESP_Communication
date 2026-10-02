#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define CE_PIN   2
#define CSN_PIN  4
#define PIN_LED  22 // GPIO conectado al ánodo del LED verde

RF24 radio(CE_PIN, CSN_PIN);

// Debe coincidir con destAddresses[0] del router
// Si eres el destino 1 usa 0x02 al final; si eres el 2 usa 0x03
const byte myAddress[5] = {0xAA, 0xAA, 0xAA, 0xAA, 0x01};

struct DataPacket {
  uint8_t targetID;
  uint8_t dataLen;
  char binaryData[16];
};

DataPacket payload;

// Misma rutina XOR que el router para descifrar en destino
void processXOR(char* data, uint8_t len) {
  const char key[] = "C1Ph3rK3y";
  int keyLen = sizeof(key) - 1;
  for (uint8_t i = 0; i < len; i++) {
    data[i] ^= key[i % keyLen];
  }
}

void printHEX(char* data, uint8_t len) {
  for (uint8_t i = 0; i < len; i++) {
    if ((uint8_t)data[i] < 0x10) Serial.print("0");
    Serial.print((uint8_t)data[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
}

void setup() {
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);

  Serial.begin(115200);
  Serial.println("\n--- Nodo Receptor (Destino 0) Activo ---");

  radio.begin();
  radio.setPALevel(RF24_PA_MIN);
  radio.setDataRate(RF24_1MBPS);

  radio.openReadingPipe(1, myAddress);
  radio.startListening();
}

void loop() {
  if (radio.available()) {
    memset(&payload, 0, sizeof(payload));
    radio.read(&payload, sizeof(payload));

    if (payload.dataLen > 15) payload.dataLen = 15;

    Serial.print("\n[RX] Paquete cifrado recibido (HEX): ");
    printHEX(payload.binaryData, payload.dataLen);

    // Descifrado simétrico
    processXOR(payload.binaryData, payload.dataLen);

    Serial.print("[RX] Mensaje descifrado: ");
    Serial.println(payload.binaryData);

    // Indicador visual de recepción
    digitalWrite(PIN_LED, HIGH);
    delay(120);
    digitalWrite(PIN_LED, LOW);
  }
}