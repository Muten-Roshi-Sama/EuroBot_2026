#include "capteur_lidar.h"
#include <Wire.h>
#include <VL53L0X.h>
#include "Debug.h"
#include "settings.h"
#include "TaskManager.h"

extern TaskManager* taskManager; // pointeur global vers TaskManager

VL53L0X capteur; // instance du capteur VL53L0X

// ===============================
// Configuration de la pin du LIDAR
// ===============================
void lidarSetupPin() {
    pinMode(LIDAR_PIN, OUTPUT);
    digitalWrite(LIDAR_PIN, HIGH); // activer le capteur
}

// ===============================
// Initialisation du LIDAR
// ===============================
bool initLidar() {
    lidarSetupPin();
    Wire.begin();

    if (!capteur.init()) {
        debugPrintf(DBG_SENSORS, "Erreur LIDAR : init échoué");
        return false;
    }

    capteur.setTimeout(500);
    capteur.startContinuous();
    debugPrintf(DBG_SENSORS, "LIDAR OK");
    return true;
}

// ===============================
// Lecture de la distance et levée du flag ISR
// ===============================
int lireDistance() {
    int d = capteur.readRangeContinuousMillimeters();
    
    if (capteur.timeoutOccurred()) return -1;
    if (d > 8190) return -2;
    
    return d; // juste retourner la distance
}

// ===============================
// Affichage état pour debug
// ===============================
void afficherEtatObstacle(int distance) {
    if (distance == -1) {
        debugPrintf(DBG_SENSORS, "ERREUR - Pas de mesure d=%d", distance);
        return;
    }

    if (distance == -2) {
        debugPrintf(DBG_SENSORS, "---- mm | Hors de portee");
        return;
    }

    debugPrintf(DBG_SENSORS, "Voie libre (%d mm)", distance);
}
