#include <ArduinoJson.h>

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  while (!Serial) ; // Attendre que le port série soit prêt (utile sur Leonardo/Micro)
}

void loop() {
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) return;
    StaticJsonDocument<256> doc;
    DeserializationError error = deserializeJson(doc, line);
    if (!error) {
      // Action sur la LED selon la commande
      if (doc["cmd"] == "LED_ON") {
        digitalWrite(LED_BUILTIN, HIGH);
      } else if (doc["cmd"] == "LED_OFF") {
        digitalWrite(LED_BUILTIN, LOW);
      }
      // Afficher le JSON reçu sur le port série
      Serial.print("[JSON reçu] : ");
      serializeJson(doc, Serial);
      Serial.println();
    } else {
      Serial.print("[Erreur JSON] : ");
      Serial.println(line);
    }
  }
}