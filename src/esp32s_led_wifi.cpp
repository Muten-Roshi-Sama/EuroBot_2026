#include <Arduino.h>
#include <WiFi.h>

const int LED_PIN = 2;
const int PORT_TCP = 8080;

const char* AP_SSID = "ESP_LED_AP";
const char* AP_PASS = "esp_password";

WiFiServer server(PORT_TCP);

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
  if (command == "LED_ON") {
    blinkEnabled = false;
    digitalWrite(LED_PIN, HIGH);
    client.println("OK: LED ON");
    return;
  }

  if (command == "LED_OFF") {
    blinkEnabled = false;
    digitalWrite(LED_PIN, LOW);
    client.println("OK: LED OFF");
    return;
  }    

  if (command.startsWith("LED_BLINK")) {
    unsigned long interval = 0;
    int sp = command.indexOf(' ');
    if (sp > 0) {
      String param = command.substring(sp + 1);
      interval = param.toInt();
    }
    if (interval >= 50) blinkInterval = interval;
    blinkEnabled = true;
    lastBlink = millis();
    ledState = digitalRead(LED_PIN);
    client.print("OK: LED BLINK ");
    client.println(blinkInterval);
    return;
  }

  client.println("ERR: Unknown command");
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  IPAddress apIP = WiFi.softAPIP();
  Serial.print("AP IP: ");
  Serial.println(apIP);

  server.begin();
}

void loop() {
  WiFiClient client = server.available();

  if (client) {
    unsigned long start = millis();
    while (!client.available() && (millis() - start) < 2000) {
      handleBlink();
      delay(1);
    }
    if (client.available()) {
      String command = client.readStringUntil('\n');
      command.trim();
      handleCommand(command, client);
    }
    client.stop();
  }

  handleBlink();
  delay(1);
}
