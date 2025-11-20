#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoJson.h>

const int LED_PIN = 2;
const int BUTTON_PIN = 9; // GPIO9, bouton BOOT sur ESP32-C3 DevKitM-1
bool lastButtonState = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;
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
  pinMode(BUTTON_PIN, INPUT_PULLUP);

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
  // Gestion bouton poussoir
  static bool buttonSent = false;
  int reading = digitalRead(BUTTON_PIN);
  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }
  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (lastButtonState == HIGH && reading == LOW && !buttonSent) {
      // Bouton pressé (transition HIGH->LOW)
      Serial.println("[DEBUG] Bouton BOOT pressé");
      if (client.connected()) {
        StaticJsonDocument<128> doc;
        doc["event"] = "button_pressed";
        doc["msg"] = "Bouton poussé";
        serializeJson(doc, client);
        client.println();
        Serial.println("[DEBUG] Message JSON envoyé au serveur");
        buttonSent = true;
      }
    }
    if (reading == HIGH) {
      buttonSent = false;
    }
  }
  lastButtonState = reading;
  delay(1);
}
