#include <WiFi.h>
#include <WiFiClient.h>
#include <ArduinoJson.h>
#include "protocol_esp32.h"

namespace protocol_esp32 {

// ------------ CONFIG WIFI -------------
const char* ssid = "PCDEFÉLIX 4208";
const char* password = "E9]3445n";

// ------------ CONFIG TCP --------------
const char* server_ip   = "10.49.106.33";
const uint16_t server_port = 8080;

// ------------ OBJETS ------------------
WiFiClient client;
HardwareSerial* uart_ptr = nullptr;

void begin_wifi_and_uart(HardwareSerial& uart, int rx, int tx, unsigned long baud) {
  uart_ptr = &uart;
  uart.begin(baud, SERIAL_8N1, rx, tx);
  delay(500);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("[WIFI] Connexion à ");
  Serial.println(ssid);
  unsigned long startAttemptTime = millis();
  const unsigned long wifiTimeout = 15000 ; // 15 secondes max
  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < wifiTimeout) {
    Serial.print(".");
    delay(1000); // Allongé pour lisibilité
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("\n[WIFI] Connecté ! IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\n[WIFI][ERREUR] Connexion impossible !");
  }
}

// --- Generalized JSON mapping system for WiFi->UART bridge ---
struct JsonMapRule {
    const char* inKey;     // received "command" value
    const char* outKey;    // output JSON key to send to Arduino
    const char* outValue;  // output JSON value
};

// Static mapping table: add new rules here
static const JsonMapRule jsonMapRules[] = {
    {"led2:on",  "led7", "on"},
    {"led2:off", "led7", "off"},
    {"pump:start", "pump", "on"},
    {"pump:stop",  "pump", "off"},
    // Add more rules as needed
};
static const size_t jsonMapRuleCount = sizeof(jsonMapRules) / sizeof(JsonMapRule);

// Applies mapping: if found, sends mapped JSON to UART and returns true
bool applyJsonMapping(const char* cmd, HardwareSerial& uart) {
    for (size_t i = 0; i < jsonMapRuleCount; ++i) {
        if (strcmp(cmd, jsonMapRules[i].inKey) == 0) {
            StaticJsonDocument<64> outDoc;
            outDoc[jsonMapRules[i].outKey] = jsonMapRules[i].outValue;
            serializeJson(outDoc, uart);
            uart.println();
            return true;
        }
    }
    return false;
}

// Main bridge loop: WiFi <-> UART, with generalized JSON mapping
void bridge_uart_wifi_loop() {
    static unsigned long lastPing = 0;
    bool sentSomething = false;
    const unsigned long interval = 2000; // 2s for ping and reconnect


    // --- Non-blocking WiFi auto-reconnect ---
    ensure_wifi_connected();
    if (WiFi.status() != WL_CONNECTED) {
      // WiFi non connecté, on ne tente rien d'autre
      return;
    }

    // --- TCP auto-reconnect logic (only if WiFi is up) ---
    if (!client.connected()) {
      client.stop();
      if (client.connect(server_ip, server_port)) {
        Serial.println("[TCP] Reconnecté au serveur !");
      } else {
        // Pas de delay bloquant, on attend le prochain tour de boucle
        static unsigned long lastTcpLog = 0;
        unsigned long now = millis();
        if (now - lastTcpLog > interval) {
          Serial.println("[TCP][ERREUR] Connexion serveur impossible, nouvelle tentative...");
          lastTcpLog = now;
        }
        return;
      }
    }

    // --- UART -> WiFi relay ---
    if (uart_ptr && uart_ptr->available()) {
        String uartStr = uart_ptr->readStringUntil('\n');
        uartStr.trim();
        if (uartStr.length() > 0) {
            client.print(uartStr + "\n");
            sentSomething = true;
        }
    }

    // --- WiFi keep-alive ping ---
    if (!sentSomething && millis() - lastPing > interval) {
        client.print("{\"ping\":1}\n");
        lastPing = millis();
    }

    // --- WiFi -> UART generalized JSON mapping ---
    if (client.available() && uart_ptr) {
        String jsonStr = client.readStringUntil('\n');
        StaticJsonDocument<128> doc;
        DeserializationError err = deserializeJson(doc, jsonStr);
        if (err == DeserializationError::Ok) {
            const char* cmd = doc["command"];
            if (cmd) {
                if (!applyJsonMapping(cmd, *uart_ptr)) {
                    Serial.print("[MAP][ERREUR] Commande inconnue: ");
                    Serial.println(cmd);
                }
            } else {
                Serial.println("[MAP][ERREUR] Champ 'command' manquant dans le JSON reçu.");
            }
        } else {
            Serial.print("[MAP][ERREUR] JSON WiFi invalide: ");
            Serial.println(jsonStr);
        }
    }
}

// --- Non-blocking WiFi reconnection helper ---
void ensure_wifi_connected() {
  static unsigned long lastAttempt = 0;
  static bool wasConnected = false;
  const unsigned long retryInterval = 5000; // 5 seconds

  wl_status_t status = WiFi.status();
  if (status == WL_CONNECTED) {
    if (!wasConnected) {
      Serial.print("[WIFI] Reconnecté ! IP: ");
      Serial.println(WiFi.localIP());
      wasConnected = true;
    }
    return;
  }
  wasConnected = false;
  unsigned long now = millis();
  if (now - lastAttempt >= retryInterval) {
    Serial.println("[WIFI] Déconnecté, tentative de reconnexion...");
    WiFi.disconnect();
    WiFi.begin(ssid, password);
    lastAttempt = now;
  }
}

} // namespace protocol_esp32

/*
// --- Exemple d'utilisation dans main.cpp ---
#include "protocol_esp32.cpp"

void setup() {
  protocol_esp32::begin_wifi_and_uart(); // Plus besoin de passer le SSID/IP ici
}

void loop() {
  protocol_esp32::bridge_uart_wifi_loop();
}

#include <WiFi.h>
#include <WiFiClient.h>
#include <ArduinoJson.h>

// ------------ CONFIG WIFI -------------
const char* ssid = "PCDEFÉLIX 4208";
const char* password = "E9]3445n";

// ------------ CONFIG TCP --------------
const char* server_ip   = "192.168.68.104";
const uint16_t server_port = 8080;

// ------------ OBJETS ------------------
WiFiClient client;

void setup() {
  Serial2.begin(115200, SERIAL_8N1, 9, 10); // UART2: RX=9 (rx1), TX=10 (tx1)
  delay(500);

  // ---- Connexion WiFi ----
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    // Optionnel: blink LED ou autre feedback matériel
  }
}

void loop() {
  // -------------------- Connexion TCP au serveur --------------------
  if (!client.connected()) {
    // Optionnel: envoyer un log sur UART2 ou LED
    client.stop(); // S'assure que la socket est bien fermée
    if (client.connect(server_ip, server_port)) {
      // Connexion rétablie
    } else {
      delay(1000);
      return;
    }
  }

  // Relaye tout message JSON reçu de l'Arduino (UART2) au PC via WiFi
  if (Serial2.available()) {
    String uartStr = Serial2.readStringUntil('\n');
    uartStr.trim();
    if (uartStr.length() > 0) {
      client.print(uartStr + "\n");
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
*/