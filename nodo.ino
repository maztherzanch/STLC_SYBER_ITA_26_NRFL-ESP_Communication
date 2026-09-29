#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define CE_PIN 2
#define CSN_PIN 4
#define LED_INDICATOR 12

RF24 radio(CE_PIN, CSN_PIN);

const byte listenAddresses[6][5] = {
  {0x11, 0x22, 0x33, 0x44, 0x55},
  {0x66, 0x77, 0x88, 0x99, 0xAA},
  {0x66, 0x77, 0x88, 0x99, 0xBB},
  {0x66, 0x77, 0x88, 0x99, 0xCC},
  {0x66, 0x77, 0x88, 0x99, 0xDD},
  {0x66, 0x77, 0x88, 0x99, 0xEE}
};

const byte destAddresses[3][5] = {
  {0xAA, 0xAA, 0xAA, 0xAA, 0x01},
  {0xAA, 0xAA, 0xAA, 0xAA, 0x02},
  {0xAA, 0xAA, 0xAA, 0xAA, 0x03}
};

struct DataPacket {
  uint8_t targetID;
  uint8_t dataLen;
  char binaryData[16];
};

DataPacket payload;

// Función XOR que solo cifra la cantidad exacta de bytes
void processXOR(char* data, uint8_t len) {
  const char key[] = "C1Ph3rK3y";
  int keyLen = sizeof(key) - 1;
  for (uint8_t i = 0; i < len; i++) {
    data[i] ^= key[i % keyLen];
  }
}

// Función para imprimir bytes cifrados en HEX sin romper la terminal
void printHEX(char* data, uint8_t len) {
  for (uint8_t i = 0; i < len; i++) {
    if ((uint8_t)data[i] < 0x10) Serial.print("0");
    Serial.print((uint8_t)data[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
}

void setup() {
  pinMode(LED_INDICATOR, OUTPUT);
  Serial.begin(115200);
  Serial.println("\n--- Nodo Enrutador Activo ---");

  radio.begin();
  radio.setPALevel(RF24_PA_MIN);
  radio.setDataRate(RF24_1MBPS);

  for(uint8_t i = 0; i < 6; i++) {
    radio.openReadingPipe(i, listenAddresses[i]);
  }
  
  radio.startListening();
}

void loop() {
  uint8_t pipeNum;
  
  if (radio.available(&pipeNum)) {
    memset(&payload, 0, sizeof(payload));
    radio.read(&payload, sizeof(payload));
    
    digitalWrite(LED_INDICATOR, HIGH);
    
    // Asegurar límite de tamaño por seguridad
    if (payload.dataLen > 15) payload.dataLen = 15;

    Serial.print("\n[ROUTER] Mensaje recibido del Pipe ");
    Serial.print(pipeNum);
    Serial.print(" -> Texto plano: ");
    Serial.println(payload.binaryData);

    // Cifrar los bytes reales
    processXOR(payload.binaryData, payload.dataLen);
    
    Serial.print("[ROUTER] Mensaje Cifrado (HEX): ");
    printHEX(payload.binaryData, payload.dataLen);

    // Reenviar
    radio.stopListening();
    
    if (payload.targetID < 3) {
      radio.openWritingPipe(destAddresses[payload.targetID]);
      
      if (radio.write(&payload, sizeof(payload))) {
        Serial.print("[ROUTER] Redirigido con exito a Destino ");
        Serial.println(payload.targetID);
      } else {
        Serial.println("[ROUTER] Error al reenviar");
      }
    }
    
    radio.startListening();
    digitalWrite(LED_INDICATOR, LOW);
  }
}
