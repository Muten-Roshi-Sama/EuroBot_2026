#include <Arduino.h>
#include <WiFi.h>

// ============================================================================
// Configuration WiFi
// ============================================================================
const char* ssid = "wifiesp";           // Remplacer par votre SSID WiFi
const char* password = "wifiesp";   // Remplacer par votre mot de passe WiFi
const int port = 8080;                   // Port du serveur TCP

// ============================================================================
// Configuration LED
// ============================================================================
const int LED_PIN = 2;  // GPIO 2 - adapter selon votre broche LED

// ============================================================================
// Serveur WiFi
// ============================================================================
WiFiServer server(port);

// ============================================================================
// Setup
// ============================================================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  // Initialiser LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Serial.println("\n\nESP32-S LED WiFi Server Starting...");
  // Start in Access Point (AP) mode so you can connect your PC directly
  // AP credentials (change if you want)
  const char* ap_ssid = "ESP_LED_AP";
  const char* ap_pass = "esp_password"; // minimum 8 characters

  WiFi.mode(WIFI_AP);
  WiFi.softAP(ap_ssid, ap_pass);
  IPAddress apIP = WiFi.softAPIP(); // usually 192.168.4.1

  Serial.print("AP SSID: ");
  Serial.println(ap_ssid);
  Serial.print("AP IP Address: ");
  Serial.println(apIP);
  Serial.print("Port: ");
  Serial.println(port);

  // Start server
  server.begin();
  Serial.println("Server started (AP mode), waiting for connections...");
}

// ============================================================================
// Loop
// ============================================================================
void loop() {
  // Vérifier les connexions entrantes
  WiFiClient client = server.available();

  if (client) {
    Serial.println("Client connected!");

    // Lire les données du client
    while (client.connected()) {
      if (client.available()) {
        String command = client.readStringUntil('\n');
        command.trim();

        Serial.print("Received: ");
        Serial.println(command);

        // Parser commande
        if (command == "LED_ON") {
          digitalWrite(LED_PIN, HIGH);
          client.println("LED is ON");
          Serial.println("LED ON");
        } else if (command == "LED_OFF") {
          digitalWrite(LED_PIN, LOW);
          client.println("LED is OFF");
          Serial.println("LED OFF");
        } else if (command == "LED_TOGGLE") {
          int currentState = digitalRead(LED_PIN);
          digitalWrite(LED_PIN, !currentState);
          client.println(currentState ? "LED is OFF" : "LED is ON");
          Serial.println(currentState ? "LED OFF" : "LED ON");
        } else if (command == "LED_STATUS") {
          int status = digitalRead(LED_PIN);
          client.print("LED Status: ");
          client.println(status ? "ON" : "OFF");
        } else {
          client.println("Unknown command. Use: LED_ON, LED_OFF, LED_TOGGLE, LED_STATUS");
        }
      }
    }

    client.stop();
    Serial.println("Client disconnected");
  }

  delay(10);
}
