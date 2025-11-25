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
  Serial.begin(115200);          // UART vers Arduino
  delay(500);

  // ---- Connexion WiFi ----
  Serial.print("Connexion au WiFi");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWiFi connectée !");
  Serial.print("IP ESP32 : ");
  Serial.println(WiFi.localIP());
}

void loop() {

  // -------------------- Connexion TCP au serveur --------------------
  if (!client.connected()) {
    Serial.print("Connexion au serveur TCP...");
    if (client.connect(server_ip, server_port)) {
      Serial.println(" OK");
    } else {
      Serial.println(" ECHEC");
      delay(1000);
      return;
    }
  }

  // -------------------- Lecture JSON venant du serveur --------------------
  if (client.connected() && client.available()) {
    String jsonStr = client.readStringUntil('\n');
    jsonStr.trim();

    if (jsonStr.length() == 0) return;

    StaticJsonDocument<256> doc;
    DeserializationError err = deserializeJson(doc, jsonStr);

    if (err) {
      Serial.println("[ERR] JSON invalide reçu du serveur");
      return;
    }

    const char* cmd = doc["command"];
    if (!cmd) return;

    // --------------------------------------------------------------------
    //       TRAITEMENT des commandes venant de l'application GUI
    // --------------------------------------------------------------------


    if (strcmp(cmd, "led2:on") == 0) {
      StaticJsonDocument<64> out;
      out["led7"] = "on";
      serializeJson(out, Serial);
      Serial.println();
      Serial.println("[UART] led7:on envoyé à l’Arduino");
    } else if (strcmp(cmd, "led2:off") == 0) {
      StaticJsonDocument<64> out;
      out["led7"] = "off";
      serializeJson(out, Serial);
      Serial.println();
      Serial.println("[UART] led7:off envoyé à l’Arduino");

    } else {
      Serial.print("[WARN] Commande inconnue : ");
      Serial.println(cmd);
    }
  }
}
