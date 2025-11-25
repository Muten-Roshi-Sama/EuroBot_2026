#include <Arduino.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include <WiFiClient.h>

namespace protocol_esp32 {

// ----------- CONFIGURATION CENTRALISÉE -------------
static const char* ssid = "PCDEFÉLIX 4208";
static const char* password = "E9]3445n";
static const char* server_ip = "192.168.137.1";
static const uint16_t server_port = 8080;

WiFiClient client;
HardwareSerial* uart_ptr = nullptr;
int uart_rx = 9;
int uart_tx = 10;

// Appeler dans setup()
void begin_wifi_and_uart(HardwareSerial& uart = Serial2, int rx = 9, int tx = 10, unsigned long baud = 115200) {
    uart_ptr = &uart;
    uart_rx = rx;
    uart_tx = tx;
    uart.begin(baud, SERIAL_8N1, rx, tx);
    delay(500);
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
    }
}

// Envoie un message JSON générique
bool send(const JsonDocument& doc, HardwareSerial& serial) {
    serializeJson(doc, serial);
    serial.println();
    return true;
}

// Reçoit un message JSON (non bloquant)
bool receive(JsonDocument& doc, HardwareSerial& serial) {
    if (serial.available()) {
        String jsonStr = serial.readStringUntil('\n');
        DeserializationError err = deserializeJson(doc, jsonStr);
        return (err == DeserializationError::Ok);
    }
    return false;
}

// Exemple d'envoi d'un événement
bool send_event(const char* key, const char* value, HardwareSerial& serial) {
    StaticJsonDocument<64> doc;
    doc[key] = value;
    return send(doc, serial);
}

// Exemple d'envoi d'un entier
bool send_int(const char* key, int value, HardwareSerial& serial) {
    StaticJsonDocument<64> doc;
    doc[key] = value;
    return send(doc, serial);
}

// Appeler dans loop()
void bridge_uart_wifi_loop() {
    // Connexion TCP automatique
    if (!client.connected()) {
        client.stop();
        client.connect(server_ip, server_port);
        delay(1000);
        return;
    }
    // UART -> PC (WiFi)
    if (uart_ptr && uart_ptr->available()) {
        String uartStr = uart_ptr->readStringUntil('\n');
        uartStr.trim();
        if (uartStr.length() > 0) {
            client.print(uartStr + "\n");
        }
    }
    // PC (WiFi) -> UART
    if (client.available() && uart_ptr) {
        String jsonStr = client.readStringUntil('\n');
        jsonStr.trim();
        if (jsonStr.length() > 0) {
            uart_ptr->print(jsonStr + "\n");
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
const char* server_ip   = "192.168.137.1";
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
}