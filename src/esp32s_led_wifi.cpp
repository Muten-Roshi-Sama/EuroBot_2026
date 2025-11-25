#include <WiFi.h>
#include <WiFiClient.h>
#include <ArduinoJson.h>

// ------------ CONFIG WIFI -------------
const char* ssid = "PCDEFÉLIX 4208";
const char* password = "E9]3445n";

// ------------ CONFIG TCP --------------
const char* server_ip   = "192.168.137.1";
const uint16_t server_port = 8080;

// ------------ OBJETS ------------------
WiFiClient client;

void setup() {
  // Pas de Serial USB
  Serial2.begin(115200, SERIAL_8N1, 9, 10); // UART2: RX=9 (rx1), TX=10 (tx1)
  delay(500);

  // ---- Connexion WiFi ----
  // Pas de Serial USB
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  // Pas de Serial USB
}

void loop() {
  // Relaye tout message JSON reçu de l'Arduino (UART2) au PC via WiFi
  if (Serial2.available()) {
    String uartStr = Serial2.readStringUntil('\n');
    uartStr.trim();
    if (uartStr.length() > 0) {
      client.print(uartStr + "\n");
    }
  }


  // -------------------- Connexion TCP au serveur --------------------
  if (!client.connected()) {
    // Log de perte de connexion envoyé au PC via WiFi
    client.print("{\"log\":\"Connexion au serveur TCP perdue !\"}\n");
    if (client.connect(server_ip, server_port)) {
      // Connexion rétablie, rien à afficher côté ESP32
    } else {
      delay(1000);
      return;
    }
  }

  // Receive from PC and send to Arduino (via UART2)
  if (client.available()) {
    String jsonStr = client.readStringUntil('\n');
    StaticJsonDocument<128> doc;
    if (deserializeJson(doc, jsonStr) == DeserializationError::Ok) {
      const char* cmd = doc["command"];
      if (cmd && strcmp(cmd, "led2:on") == 0) {
        StaticJsonDocument<32> outDoc;
        outDoc["led7"] = "on";
        serializeJson(outDoc, Serial2);
        Serial2.println();
      } else if (cmd && strcmp(cmd, "led2:off") == 0) {
        StaticJsonDocument<32> outDoc;
        outDoc["led7"] = "off";
        serializeJson(outDoc, Serial2);
        Serial2.println();
      }
    }
  }
}
