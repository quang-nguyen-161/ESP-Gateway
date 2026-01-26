/*
  ESP32 LoRa Gateway → ThingsBoard Gateway
  - Auto-create devices (nodes)
  - Send telemetry per node
*/

#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <time.h>

/* ===================== WiFi ===================== */
#define WIFI_SSID       "Xuaaan"
#define WIFI_PASSWORD   "quang1612004"

/* ================= ThingsBoard ================== */
#define TB_SERVER       "demo.thingsboard.io"
#define TB_PORT         1883
#define TB_TOKEN        "sj5c5r71ovm0i09079fg"   // TOKEN OF GATEWAY DEVICE

/* ================= UART (LoRa) ================== */
#define RX_PIN 16
#define TX_PIN 17

/* ================= Protocol ===================== */
#define PACKET_SIZE 9
#define MAX_NODES   10
#define RX_BUF_SIZE 255
#define NODES_PER_TX 2

/* ================= Globals ====================== */
WiFiClient wifiClient;
PubSubClient client(wifiClient);
HardwareSerial esp_serial(2);

uint8_t rxBuf[RX_BUF_SIZE];

/* ================= Data Structure =============== */
struct sensorData {
  String dev_id;
  float temperature;
  float humidity;
  float soil_moisture;
  float battery;
};

/* ================= Function Prototypes ========= */
void connectWiFi();
void connectMQTT();
int parse_data(uint8_t *buf, int len, sensorData *nodes, int maxNodes);
void mqttCallback(char* topic, byte* payload, unsigned int length);

void sensorsToGatewayTB(sensorData *data, int count);

/* ================= Setup ======================== */
void setup() {
  Serial.begin(115200);
  esp_serial.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);

  connectWiFi();
    client.setCallback(mqttCallback); 
  connectMQTT();
  

  configTime(0, 0, "pool.ntp.org", "time.nist.gov");

  time_t now;

Serial.print("Waiting for NTP time");
do {
  delay(500);
  Serial.print(".");
  time(&now);
} while (now < 100000);   // time not valid yet

Serial.println("\nTime synchronized");


  Serial.println("ESP32 ThingsBoard Gateway Ready");
}

/* ================= Main Loop ==================== */
void loop() {
  if (!client.connected()) {
    connectMQTT();
  }
  client.loop();

  if (esp_serial.available() >= PACKET_SIZE) {

    int len = esp_serial.readBytes(rxBuf, RX_BUF_SIZE);

    Serial.print("Received HEX: ");
    for (int i = 0; i < len; i++) {
      Serial.printf("%02X ", rxBuf[i]);
    }
    Serial.println();

    sensorData nodes[MAX_NODES];
    int nodeCount = parse_data(rxBuf, len, nodes, MAX_NODES);

    if (nodeCount > 0)
{
  for (int i = 0; i < nodeCount; i += NODES_PER_TX)
  {
    int batchSize = min(NODES_PER_TX, nodeCount - i);

    sensorsToGatewayTB(&nodes[i], batchSize);

    delay(200);   // small gap between MQTT publishes
  }
}
  }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  Serial.println("\n===== DATA FROM THINGSBOARD =====");
  Serial.print("Topic: ");
  Serial.println(topic);

  String msg;
  for (unsigned int i = 0; i < length; i++) {
    msg += (char)payload[i];
  }
  Serial.print("Payload: ");
  Serial.println(msg);

  StaticJsonDocument<256> doc;
  if (deserializeJson(doc, msg)) {
    Serial.println("JSON parse error");
    return;
  }

  JsonObject data;

  // ✅ Handle both UPDATE and RESPONSE
  if (doc.containsKey("shared")) {
    data = doc["shared"].as<JsonObject>();
  } else {
    data = doc.as<JsonObject>();
  }

  if (data.containsKey("state")) {
    int state = data["state"];
    Serial.print("state = ");
    Serial.println(state);
  }

  if (data.containsKey("set_values")) {
    float setValues = data["set_values"];
    Serial.print("set_values = ");
    Serial.println(setValues);
  }

  Serial.println("================================\n");
}


#define CMD_FORWARD  0x20
#define CMD_NORMAL   0x02
#define MAX_CHILD    10

int parse_data(uint8_t *buf, int len, sensorData *nodes, int maxNodes)
{
    int i = 0;
    int count = 0;

    while (i < len && count < maxNodes)
    {
        /* ================= NORMAL PACKET ================= */
        // Format:
        // 02 FF node slot frame [8B]
        if (buf[i] == CMD_NORMAL)
        {
            if (i + 13 > len) break;

            nodes[count].dev_id = buf[i + 2];  // node id

            int d = i + 5;

            uint16_t t = (buf[d]     << 8) | buf[d + 1];
            uint16_t h = (buf[d + 2] << 8) | buf[d + 3];
            uint16_t s = (buf[d + 4] << 8) | buf[d + 5];
            uint16_t b = (buf[d + 6] << 8) | buf[d + 7];

            nodes[count].temperature   = t / 100.0f;
            nodes[count].humidity      = h / 100.0f;
            nodes[count].soil_moisture = s / 100.0f;
            nodes[count].battery       = b / 100.0f;

            count++;
            i += 13;
        }

        /* ================= FORWARD PACKET ================= */
        // Format:
        // 20 FF parent
        // [20 parent child slot frame 8B] x N
        else if (buf[i] == CMD_FORWARD)
        {
            if (i + 3 > len) break;

            uint8_t parent = buf[i + 2];
            i += 3;   // move to first child

            int child_count = 0;

            while ((i + 13) <= len &&
                   buf[i] == CMD_FORWARD &&
                   child_count < MAX_CHILD &&
                   count < maxNodes)
            {
                nodes[count].dev_id = buf[i + 2];  // child id

                int d = i + 5;

                uint16_t t = (buf[d]     << 8) | buf[d + 1];
                uint16_t h = (buf[d + 2] << 8) | buf[d + 3];
                uint16_t s = (buf[d + 4] << 8) | buf[d + 5];
                uint16_t b = (buf[d + 6] << 8) | buf[d + 7];

                nodes[count].temperature   = t / 100.0f;
                nodes[count].humidity      = h / 100.0f;
                nodes[count].soil_moisture = s / 100.0f;
                nodes[count].battery       = b / 100.0f;

                count++;
                child_count++;
                i += 13;
            }
        }

        /* ================= UNKNOWN BYTE ================= */
        else
        {
            i++;  // resync
        }
    }

    return count;
}





void sensorsToGatewayTB(sensorData *data, int count)
{
  StaticJsonDocument<2048> doc;

  // If ESP32 has internet → time(NULL) is OK
  uint64_t ts = (uint64_t)time(NULL) * 1000ULL;

  for (int i = 0; i < count; i++)
  {
    String deviceName = "Node_" + String(data[i].dev_id);

    JsonArray arr = doc.createNestedArray(deviceName);
    JsonObject obj = arr.createNestedObject();

    obj["ts"] = ts;

    JsonObject values = obj.createNestedObject("values");
    values["temperature"] = data[i].temperature;
    values["humidity"]    = data[i].humidity;
    values["soil"]        = data[i].soil_moisture;
    values["battery"]     = data[i].battery;
  }

  String payload;
  serializeJson(doc, payload);

  Serial.println("Sending Gateway Payload:");
  Serial.println(payload);

  client.publish("v1/gateway/telemetry", payload.c_str());
}


/* ================= WiFi Connect ================= */
void connectWiFi() {
  Serial.print("Connecting to WiFi");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
}

/* ================= MQTT Connect ================= */
void connectMQTT() {
  client.setServer(TB_SERVER, TB_PORT);

 
  while (!client.connected()) {
    Serial.print(".");
    if (client.connect("ESP32_Gateway", TB_TOKEN, NULL)) {
     

      // ✅ attribute update
      client.subscribe("v1/devices/me/attributes");

      // ✅ attribute response (THIS WAS MISSING)
      client.subscribe("v1/devices/me/attributes/response/+");

      Serial.println("Subscribed to attribute topics");

      // ✅ request current attributes
      client.publish(
        "v1/devices/me/attributes/request/1",
        "{\"sharedKeys\":\"state,set_values\"}"
      );

    } else {
      Serial.print("\nFailed, rc=");
      Serial.print(client.state());
      Serial.println(" retry in 3s");
      delay(3000);
    }
  }
}


