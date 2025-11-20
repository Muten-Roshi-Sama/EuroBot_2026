#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoJson.h>

const int LED_PIN = 2;
const int PORT_TCP = 8080;


const char* WIFI_SSID = "PCDEFÉLIX 4208"; // SSID du hotspot PC
const char* WIFI_PASS = "E9]3445n";       // Mot de passe du hotspot PC
const char* SERVER_IP = "192.168.137.1";  // IP du PC (à adapter si besoin)

WiFiClient client;

bool blinkEnabled = false;
unsigned long blinkInterval = 500;
unsigned long lastBlink = 0;
int ledState = LOW;

void handleBlink() {
  if (!blinkEnabled) return;
  unsigned long now = millis();
  if (now - lastBlink >= blinkInterval) {
    lastBlink = now;
    ledState = !ledState;
    digitalWrite(LED_PIN, ledState);
  }
}

void handleCommand(const String &command, WiFiClient &client) {
  StaticJsonDocument<256> doc;
  DeserializationError error = deserializeJson(doc, command);
  StaticJsonDocument<256> response;

  if (error) {
    response["status"] = "ERR";
    response["msg"] = "Invalid JSON";
    serializeJson(response, client);
    client.println();
    return;
  }

  String cmd = doc["cmd"] | "";
  if (cmd == "LED_ON") {
    blinkEnabled = false;
    digitalWrite(LED_PIN, HIGH);
    response["status"] = "OK";
    response["msg"] = "LED ON";
    serializeJson(response, client);
    client.println();
    return;
  }
  if (cmd == "LED_OFF") {
    blinkEnabled = false;
    digitalWrite(LED_PIN, LOW);
    response["status"] = "OK";
    response["msg"] = "LED OFF";
    serializeJson(response, client);
    client.println();
    return;
  }
  if (cmd == "LED_BLINK") {
    unsigned long interval = doc["interval"] | 500;
    if (interval >= 50) blinkInterval = interval;
    blinkEnabled = true;
    lastBlink = millis();
    ledState = digitalRead(LED_PIN);
    response["status"] = "OK";
    response["msg"] = "LED BLINK";
    response["interval"] = blinkInterval;
    serializeJson(response, client);
    client.println();
    return;
  }
  response["status"] = "ERR";
  response["msg"] = "Unknown command";
  serializeJson(response, client);
  client.println();
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connexion au WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("Connecté, IP ESP32: ");
  Serial.println(WiFi.localIP());
}

void loop() {
  static bool connected = false;
  if (!connected) {
    Serial.print("Connexion au serveur TCP...");
    if (client.connect(SERVER_IP, PORT_TCP)) {
      Serial.println(" OK");
      connected = true;
    } else {
      Serial.println(" ECHEC");
      delay(2000);
      return;
    }
  }

  // Si connecté, lire les commandes du serveur Python
  if (client.connected() && client.available()) {
    String command = client.readStringUntil('\n');
    command.trim();
    handleCommand(command, client);
  }

  handleBlink();
  delay(1);
}
