
#include <WiFi.h>
#include <WiFiClient.h>
#include <ArduinoJson.h>
#include "protocol_esp32.h"

namespace protocol_esp32 {

// ------------ CONFIG WIFI -------------
const char* ssid = "PCDEFÉLIX 4208";
const char* password = "E9]3445n";

// ------------ CONFIG TCP --------------
const char* server_ip   = "192.168.68.104";
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
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
}

void bridge_uart_wifi_loop() {
  if (!client.connected()) {
    client.stop();
    if (!client.connect(server_ip, server_port)) {
      delay(1000);
      return;
    }
  }

  if (uart_ptr && uart_ptr->available()) {
    String uartStr = uart_ptr->readStringUntil('\n');
    uartStr.trim();
    if (uartStr.length() > 0) {
      client.print(uartStr + "\n");
    }
  }

  if (client.available() && uart_ptr) {
    String jsonStr = client.readStringUntil('\n');
    StaticJsonDocument<128> doc;
    if (deserializeJson(doc, jsonStr) == DeserializationError::Ok) {
      const char* cmd = doc["command"];
      if (cmd && strcmp(cmd, "led2:on") == 0) {
        StaticJsonDocument<32> outDoc;
        outDoc["led7"] = "on";
        serializeJson(outDoc, *uart_ptr);
        uart_ptr->println();
      } else if (cmd && strcmp(cmd, "led2:off") == 0) {
        StaticJsonDocument<32> outDoc;
        outDoc["led7"] = "off";
        serializeJson(outDoc, *uart_ptr);
        uart_ptr->println();
      }
    }
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