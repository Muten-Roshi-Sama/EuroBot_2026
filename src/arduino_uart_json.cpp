#include <ArduinoJson.h>

void setup() {
  Serial.begin(115200);
  pinMode(7, OUTPUT);
  digitalWrite(7, LOW);
  pinMode(4, INPUT_PULLUP);
}

void loop() {
  // LED control via UART JSON
  if (Serial.available()) {
    String jsonStr = Serial.readStringUntil('\n');
    StaticJsonDocument<64> doc;
    if (deserializeJson(doc, jsonStr) == DeserializationError::Ok) {
      const char* ledCmd = doc["led7"];
      if (ledCmd) {
        if (strcmp(ledCmd, "on") == 0) {
          digitalWrite(7, HIGH);
          StaticJsonDocument<64> logDoc;
          logDoc["log"] = "LED ON";
          serializeJson(logDoc, Serial);
          Serial.println();
          Serial.flush();
        } else if (strcmp(ledCmd, "off") == 0) {
          digitalWrite(7, LOW);
          StaticJsonDocument<64> logDoc;
          logDoc["log"] = "LED OFF";
          serializeJson(logDoc, Serial);
          Serial.println();
          Serial.flush();
        }
      }
    }
  }
  // Button on pin 4: send JSON when pressed
  static bool lastButton = HIGH;
  bool button = digitalRead(4);
  if (lastButton == HIGH && button == LOW) {
    StaticJsonDocument<64> doc;
    doc["button4"] = "pressed";
    serializeJson(doc, Serial);
    Serial.println();
    Serial.flush();
  }
  lastButton = button;
}