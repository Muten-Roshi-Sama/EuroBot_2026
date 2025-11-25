#include <ArduinoJson.h>

void setup() {
  Serial.begin(115200);
  pinMode(7, OUTPUT);
  digitalWrite(7, LOW);
}

void loop() {
  if (Serial.available()) {
    String jsonStr = Serial.readStringUntil('\n');
    StaticJsonDocument<32> doc;
    if (deserializeJson(doc, jsonStr) == DeserializationError::Ok) {
      const char* ledCmd = doc["led7"];
      if (ledCmd) {
        if (strcmp(ledCmd, "on") == 0) digitalWrite(7, HIGH);
        else if (strcmp(ledCmd, "off") == 0) digitalWrite(7, LOW);
      }
    }
  }
}