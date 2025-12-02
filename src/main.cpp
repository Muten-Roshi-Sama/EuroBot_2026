#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>

// Nombre de capteurs
constexpr uint8_t N_LIDARS = 3;

// XSHUT pins pour chaque capteur (D2, D3, D4)
const uint8_t xshutPins[N_LIDARS] = {2, 3, 4};

// Adresses I2C assignées (après réinitialisation)
const uint8_t lidarAddresses[N_LIDARS] = {0x30, 0x31, 0x32};

VL53L0X sensors[N_LIDARS];
bool sensorAlive[N_LIDARS] = {false, false, false};

void tcaDelaySmall() {
  // petit délai pour la stabilisation I2C après changement XSHUT
  delay(50);
}

void setup() {
  Serial.begin(115200);
  delay(800); // laisse le temps au moniteur série de se connecter
  Serial.println("=== Setup démarré ===");
  Wire.begin();

  // Mettre tous les XSHUT LOW pour forcer reset
  for (uint8_t i = 0; i < N_LIDARS; ++i) {
    pinMode(xshutPins[i], OUTPUT);
    digitalWrite(xshutPins[i], LOW);
  }
  delay(50);

  // Initialiser chaque capteur un par un
  for (uint8_t i = 0; i < N_LIDARS; ++i) {
    Serial.print("Activation capteur ");
    Serial.println(i);

    // Activer uniquement ce capteur
    digitalWrite(xshutPins[i], HIGH);
    tcaDelaySmall();

    // Tentative d'init
    if (!sensors[i].init()) {
      Serial.print("⚠ Erreur init capteur ");
      Serial.println(i);
      sensorAlive[i] = false;
      // Laisser XSHUT HIGH (ou LOW selon ton choix) ; ici on laisse HIGH pour tester plus tard
    } else {
      // Changer l'adresse par défaut 0x29 -> nouvelle adresse
      sensors[i].setAddress(lidarAddresses[i]);
      sensorAlive[i] = true;
      Serial.print("Capteur ");
      Serial.print(i);
      Serial.print(" initialisé -> adresse 0x");
      Serial.println(lidarAddresses[i], HEX);

      // Optionnel : réglages supplémentaires
      sensors[i].setTimeout(500);
      // Démarrer en mode continu pour lecture rapide
      sensors[i].startContinuous();
    }

    // Laisser le temps au bus et au capteur
    delay(50);
  }

  // Afficher résumé
  Serial.println("=== Initialisation terminée ===");
  for (uint8_t i = 0; i < N_LIDARS; ++i) {
    Serial.print("Capteur ");
    Serial.print(i);
    Serial.print(" : ");
    Serial.println(sensorAlive[i] ? "OK" : "KO");
  }
  Serial.println("-------------------------------");
}

void loop() {
  // Lire chaque capteur si vivant
  for (uint8_t i = 0; i < N_LIDARS; ++i) {
    if (!sensorAlive[i]) {
      Serial.print("Capteur ");
      Serial.print(i);
      Serial.println(" non initialisé -> skip");
      continue;
    }

    // Lecture continue (retourne millimètres)
    uint16_t dist = sensors[i].readRangeContinuousMillimeters();

    if (sensors[i].timeoutOccurred()) {
      Serial.print("⚠ Timeout capteur ");
      Serial.println(i);
    } else {
      Serial.print("Lidar ");
      Serial.print(i);
      Serial.print(" (0x");
      Serial.print(lidarAddresses[i], HEX);
      Serial.print(") = ");
      Serial.print(dist);
      Serial.println(" mm");
    }
    // petit gap entre lectures pour le bus
    delay(30);
  }

  Serial.println("-------------------------------");
  delay(1000);
}
