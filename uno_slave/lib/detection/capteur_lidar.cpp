#include "capteur_lidar.h"
#include <Wire.h>
#include <VL53L0X.h>
#include "Debug.h"
#include "settings.h"

VL53L0X lidar1; // capteur 1
VL53L0X lidar2; // capteur 2
VL53L0X lidar3; // capteur 3


// ===============================
// Configuration des pins XSHUT
// ===============================
void lidarSetupPins() {
    pinMode(LIDAR1_PIN, OUTPUT);
    pinMode(LIDAR2_PIN, OUTPUT);
    pinMode(LIDAR3_PIN, OUTPUT);
    
    // Désactiver les 3 capteurs au départ
    digitalWrite(LIDAR1_PIN, LOW);
    digitalWrite(LIDAR2_PIN, LOW);
    digitalWrite(LIDAR3_PIN, LOW);
    delay(10);
}

// ===============================
// Initialisation des 3 LIDARs
// ===============================
bool initLidar() {
    lidarSetupPins();
    Wire.begin();
    delay(200);

    digitalWrite(LIDAR1_PIN, LOW);
    digitalWrite(LIDAR2_PIN, LOW);
    digitalWrite(LIDAR3_PIN, LOW);
    delay(100);

    // LIDAR1
    digitalWrite(LIDAR1_PIN, HIGH);
    delay(200);
    lidar1.setTimeout(500);
    if (!lidar1.init()) return false;
    lidar1.setAddress(0x30);
    delay(50);

    // LIDAR2
    digitalWrite(LIDAR2_PIN, HIGH);
    delay(200);
    lidar2.setTimeout(500);
    if (!lidar2.init()) return false;
    lidar2.setAddress(0x31);
    delay(50);

    // LIDAR3
    digitalWrite(LIDAR3_PIN, HIGH);
    delay(200);
    lidar3.setTimeout(500);
    if (!lidar3.init()) return false;
    delay(50);

    // Démarrer mode continu
    lidar1.startContinuous(100); // réduit à 100ms au lieu de 50ms
    lidar2.startContinuous(100);
    lidar3.startContinuous(100);
    
    return true;
}
// ===============================
// Lecture LIDAR1
// ===============================
int lireDistanceLidar1() {
    int d = lidar1.readRangeContinuousMillimeters();
    if (lidar1.timeoutOccurred()) return -1;
    if (d > 8190) return -2;
    return d;
}

// ===============================
// Lecture LIDAR2
// ===============================
int lireDistanceLidar2() {
    int d = lidar2.readRangeContinuousMillimeters();
    if (lidar2.timeoutOccurred()) return -1;
    if (d > 8190) return -2;
    return d;
}

// ===============================
// Lecture LIDAR3
// ===============================
int lireDistanceLidar3() {
    int d = lidar3.readRangeContinuousMillimeters();
    if (lidar3.timeoutOccurred()) return -1;
    if (d > 8190) return -2;
    return d;
}

// ===============================
// Affichage état pour debug
// ===============================
void afficherEtatObstacle(int distance) {
    if (distance == -1) {
        debugPrintf(DBG_SENSORS, "ERREUR - Pas de mesure");
        return;
    }
    if (distance == -2) {
        debugPrintf(DBG_SENSORS, "Hors de portée");
        return;
    }
    debugPrintf(DBG_SENSORS, "Voie libre (%d mm)", distance);
}