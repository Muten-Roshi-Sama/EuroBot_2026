#include <Wire.h>
#include "Adafruit_APDS9960.h"

// --- Capteur couleur ---
Adafruit_APDS9960 apds;

// --- LED d’éclairage ---
const int ledPin = 9;

// --- Capteur ultrason ---
const int trigPin = 6;
const int echoPin = 7;

// --- Calibration des couleurs ---
struct Color {
  int r, g, b;
};

Color jaune, bleu, noir;

// --- Variables ---
int r, g, b, c;
long duration;
float distance;
const float seuilDetection = 10.0; // distance en cm

void setup() {
  Serial.begin(9600);
  Wire.begin();

  // Initialisation APDS9960
  if (!apds.begin()) {
    Serial.println("Erreur : capteur APDS9960 non détecté !");
    while (1);
  }
  apds.enableColor(true);

  // LED
  pinMode(ledPin, OUTPUT);
  analogWrite(ledPin, 0); // LED éteinte au démarrage

  // Ultrason
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  Serial.println("=== Calibration ===");
  Serial.println("Place le capteur sur du JAUNE et appuie sur 'j'");
  Serial.println("Puis sur du BLEU ('b') et du NOIR ('n')");
}

void loop() {
  // Lecture du capteur ultrason
  distance = getDistance();

  // Seulement si un objet est proche
  if (distance > 0 && distance < seuilDetection) {
    analogWrite(ledPin, 180); // Allume la LED d’éclairage

    // Lecture des couleurs
    uint16_t r16, g16, b16, c16;
    if (apds.colorDataReady()) {
      apds.getColorData(&r16, &g16, &b16, &c16);
      r = r16; g = g16; b = b16; c = c16;

      // Calibration manuelle via le moniteur série
      if (Serial.available()) {
        char cmd = Serial.read();
        if (cmd == 'j') { jaune = {r, g, b}; Serial.println("Jaune calibré."); }
        if (cmd == 'b') { bleu = {r, g, b}; Serial.println("Bleu calibré."); }
        if (cmd == 'n') { noir = {r, g, b}; Serial.println("Noir calibré."); }
      }

      // Détection de couleur
      String couleur = detecterCouleur(r, g, b);

      // Affichage
      Serial.print("Distance: ");
      Serial.print(distance);
      Serial.print(" cm | Couleur: ");
      Serial.println(couleur);
    }
  } else {
    analogWrite(ledPin, 0); // Éteint la LED si rien devant
  }

  delay(200);
}

String detecterCouleur(int r, int g, int b) {
  if (jaune.r == 0 && bleu.r == 0 && noir.r == 0) return "Non calibré";

  float dJaune = distanceCouleur(r, g, b, jaune);
  float dBleu = distanceCouleur(r, g, b, bleu);
  float dNoir = distanceCouleur(r, g, b, noir);

  float minD = min(dJaune, min(dBleu, dNoir));
  if (minD == dJaune) return "Jaune";
  if (minD == dBleu) return "Bleu";
  if (minD == dNoir) return "Noir";
  return "Inconnu";
}

float distanceCouleur(int r, int g, int b, Color ref) {
  return sqrt(pow(r - ref.r, 2) + pow(g - ref.g, 2) + pow(b - ref.b, 2));
}

float getDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  if (duration == 0) return -1; // pas de mesure valide
  return duration * 0.034 / 2;
}
