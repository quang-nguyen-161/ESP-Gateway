/*
  LoRa Simple Gateway Example - ESP32 + SX1278
  SPI speed: 100 kHz
  LoRa frequency: 434 MHz
  Gateway logic: Tx InvertIQ enabled, Rx InvertIQ disabled
*/

#include <SPI.h>
#include <LoRa.h>
#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// WiFi
#define WIFI_SSID       "HaThanhv2"
#define WIFI_PASSWORD   "88889999"

// ThingsBoard MQTT Server
#define TB_SERVER       "demo.thingsboard.io"
#define TB_PORT         1883
#define TB_TOKEN        "g1hea96r0c6l876plbkh"   

WiFiClient wifiClient;
PubSubClient client(wifiClient);
String ack =(String) 0xFF;
String payload = "";
bool LoraMessageReceived = false;

// LoRa pins
const int csPin    = 17;  // NSS/CS
const int resetPin = 4;   // RESET
const int irqPin   = 2;   // DIO0 (IRQ)

// SPI pins
#define SCK_PIN  18
#define MISO_PIN 19
#define MOSI_PIN 5

// LoRa frequency
const long frequency = 433E6; // 433 MHz

// Custom SPI object
SPIClass mySPI(VSPI);

void connectWiFi();
void connectMQTT();

void LoRa_rxMode();
void LoRa_txMode();
void LoRa_sendMessage(String message);
void onReceive(int packetSize);
void onTxDone();
boolean runEvery(unsigned long interval);
String parseToJson(String dataStr);

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println("LoRa Gateway Init...");

  // Initialize custom SPI pins
  mySPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN);

  // Set LoRa pins and SPI object
  LoRa.setPins(csPin, resetPin, irqPin);
  LoRa.setSPI(mySPI);
  // Set frequency
  LoRa.setFrequency(433E6); // 433 MHz

  // Set TX power
  LoRa.setTxPower(20); // 20 dBm

  // LoRa parameters
  LoRa.setSpreadingFactor(7);   // SF7
  LoRa.setSignalBandwidth(125E3); // 125 kHz
  LoRa.setCodingRate4(5);       // 4/5
  LoRa.enableCrc();


  Serial.println("LoRa initialized!");

  // Initialize LoRa
  if (!LoRa.begin(frequency)) {
    Serial.println("LoRa init failed! Check wiring.");
    while (true);
  }

  Serial.println("LoRa init succeeded.");
  connectWiFi();
  connectMQTT();


  // LoRa callbacks
  LoRa.onReceive(onReceive);
  LoRa.onTxDone(onTxDone);

  // Start in receive mode
  LoRa_rxMode();
  
}

void loop() {
  // Send message every 5 seconds
  //client.loop();
 
}

// Switch LoRa to receive mode
void LoRa_rxMode() {
  LoRa.disableInvertIQ(); // Normal IQ for receiving nodes
  LoRa.receive();         // Set LoRa to receive
}

// Switch LoRa to transmit mode
void LoRa_txMode() {
  LoRa.idle();            // Standby
  LoRa.disableInvertIQ();  // Invert IQ for Gateway transmit
}

// Send a string message via LoRa
void LoRa_sendMessage(String message) {
  LoRa_txMode();           // Prepare TX
  LoRa.beginPacket();
  LoRa.print(message);
  Serial.println("Sending LoRa message: " + message);
  LoRa.endPacket(true);    // Send packet
}

// Callback when a packet is received
void onReceive(int packetSize) {
  String message = "";
  while (LoRa.available()) {
    message += (char)LoRa.read();
  }

 Serial.print("Gateway Receive: ");
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

void connectWiFi() {
  Serial.print("Connecting to WiFi");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println("\nWiFi connected.");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
}

void connectMQTT() {
  client.setServer(TB_SERVER, TB_PORT);

  Serial.print("Connecting to ThingsBoard MQTT");
  while (!client.connected()) {
    Serial.print(".");
    if (client.connect("ESP32_Client", TB_TOKEN, NULL)) {
      Serial.println("\nConnected to ThingsBoard!");
    } else {
      Serial.print("Failed, rc=");
      Serial.print(client.state());
      Serial.println(" retry in 3s");
      delay(3000);
    }
  }
}

String parseToJson(String dataStr) {
  JsonDocument doc;
  String values[5] = {"0","0","0","0","0"};

  int index = 0;
  int lastIndex = 0;
  
  for (int i = 0; i < dataStr.length() && index < 5; i++) {
    if (dataStr[i] == ',') {
      values[index++] = dataStr.substring(lastIndex, i);
      lastIndex = i + 1;
    }
  }
  
  if (index < 5) values[index] = dataStr.substring(lastIndex);

  doc["dev_id"] = values[0];
  doc["temp"] = values[1];
  doc["humidity"] = values[2];
  doc["soil_moisture"] = values[3];
  doc["battery"] = values[4];

  String jsonStr;
  serializeJson(doc, jsonStr);
  return jsonStr;
}
