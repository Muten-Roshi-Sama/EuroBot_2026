#include <Arduino.h>
#include <ArduinoJson.h>

// Protocole UART générique pour Arduino <-> ESP32 (ou autre)
// Cette librairie gère l'envoi et la réception de messages JSON bien formés sur n'importe quel port série
// Usage : voir exemple en bas de fichier

namespace protocol {

// Initialisation du port série (par défaut Serial)
void init(HardwareSerial& serial = Serial, unsigned long baudrate = 115200) {
    serial.begin(baudrate);
}

// Envoie un message JSON générique (clé/valeur)
bool send(const JsonDocument& doc, HardwareSerial& serial = Serial) {
    serializeJson(doc, serial);
    serial.println();
    return true;
}

// Envoie un événement simple (clé/valeur string)
bool send_event(const char* key, const char* value, HardwareSerial& serial = Serial) {
    StaticJsonDocument<64> doc;
    doc[key] = value;
    return send(doc, serial);
}

// Envoie un entier (clé/valeur int)
bool send_int(const char* key, int value, HardwareSerial& serial = Serial) {
    StaticJsonDocument<64> doc;
    doc[key] = value;
    return send(doc, serial);
}

// Vérifie l'état d'un bouton (LOW = pressé) et envoie un événement si pressé
void check_button(uint8_t button_pin, const char* event_key = "button", const char* event_value = "pressed", HardwareSerial& serial = Serial) {
    static bool lastButton = HIGH;
    bool button = digitalRead(button_pin);
    if (lastButton == HIGH && button == LOW) {
        send_event(event_key, event_value, serial);
    }
    lastButton = button;
}

// Tente de lire un message JSON reçu sur le port série donné
// Retourne true si un message valide a été reçu et parsé
bool receive(JsonDocument& doc, HardwareSerial& serial = Serial) {
    if (serial.available()) {
        String jsonStr = serial.readStringUntil('\n');
        DeserializationError err = deserializeJson(doc, jsonStr);
        return (err == DeserializationError::Ok);
    }
    return false;
}

} // namespace protocol

/*
// --- Exemple d'ancien code de communication (avant librairie) ---

void loop() {
    // LED control via UART JSON
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
    // Button on pin 4: send JSON when pressed
    static bool lastButton = HIGH;
    bool button = digitalRead(4);
    if (lastButton == HIGH && button == LOW) {
        StaticJsonDocument<32> doc;
        doc["button4"] = "pressed";
        serializeJson(doc, Serial);
        Serial.println();
    }
    lastButton = button;
}

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


*/
