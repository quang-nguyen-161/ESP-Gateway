/*
  LoRa Simple Gateway Example - ESP32 + SX1278
  SPI speed: 100 kHz
  LoRa frequency: 434 MHz
  Gateway logic: Tx InvertIQ enabled, Rx InvertIQ disabled
*/

#include <SPI.h>
#include <LoRa.h>

// LoRa pins
const int csPin    = 17;  // NSS/CS
const int resetPin = 4;   // RESET
const int irqPin   = 2;   // DIO0 (IRQ)

// SPI pins
#define SCK_PIN  18
#define MISO_PIN 19
#define MOSI_PIN 5

// LoRa frequency
const long frequency = 434E6; // 434 MHz

// Custom SPI object
SPIClass mySPI(VSPI);

void LoRa_rxMode();
void LoRa_txMode();
void LoRa_sendMessage(String message);
void onReceive(int packetSize);
void onTxDone();
boolean runEvery(unsigned long interval);

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println("LoRa Gateway Init...");

  // Initialize custom SPI pins
  mySPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN);

  // Set LoRa pins and SPI object
  LoRa.setPins(csPin, resetPin, irqPin);
  LoRa.setSPI(mySPI);
  LoRa.setSPIFrequency(100000); // 100 kHz SPI speed

  // Initialize LoRa
  if (!LoRa.begin(frequency)) {
    Serial.println("LoRa init failed! Check wiring.");
    while (true);
  }

  Serial.println("LoRa init succeeded.");

  // LoRa callbacks
  LoRa.onReceive(onReceive);
  LoRa.onTxDone(onTxDone);

  // Start in receive mode
  LoRa_rxMode();
}

void loop() {
  // Send message every 5 seconds
  if (runEvery(5000)) {
    String message = "HeLoRa World! ";
    message += "I'm a Gateway! ";
    message += millis();

    LoRa_sendMessage(message);
    Serial.println("Message sent!");
  }
}

// Switch LoRa to receive mode
void LoRa_rxMode() {
  LoRa.disableInvertIQ(); // Normal IQ for receiving nodes
  LoRa.receive();         // Set LoRa to receive
}

// Switch LoRa to transmit mode
void LoRa_txMode() {
  LoRa.idle();            // Standby
  LoRa.enableInvertIQ();  // Invert IQ for Gateway transmit
}

// Send a string message via LoRa
void LoRa_sendMessage(String message) {
  LoRa_txMode();           // Prepare TX
  LoRa.beginPacket();
  LoRa.print(message);
  LoRa.endPacket(true);    // Send packet
}

// Callback when a packet is received
void onReceive(int packetSize) {
  String message = "";
  while (LoRa.available()) {
    message += (char)LoRa.read();
  }
  Serial.print("Received: ");
  Serial.println(message);
}

// Callback when TX is done
void onTxDone() {
  Serial.println("TxDone");
  LoRa_rxMode(); // Return to receive mode
}

// Helper: run something at intervals
boolean runEvery(unsigned long interval) {
  static unsigned long previousMillis = 0;
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    return true;
  }
  return false;
}