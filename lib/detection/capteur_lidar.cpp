#include "capteur_lidar.h"
#include <Wire.h>
#include <VL53L0X.h>
#include "Debug.h"
#include "settings.h"
#include "TaskManager.h"

extern TaskManager* taskManager;   // pointeur global vers TaskManager
VL53L0X capteur;                   // instance du capteur VL53L0X


// ===============================
// Configuration de la pin du LIDAR
// ===============================
void lidarSetupPin() {
    //pinMode(LIDAR_PIN, OUTPUT);
    //digitalWrite(LIDAR_PIN, HIGH); // activer le capteur
}


// ===============================
// Initialisation du LIDAR
// ===============================
bool initLidar() {
    lidarSetupPin();
    Wire.begin();
    delay(200);

    debugPrintf(DBG_SENSORS, "Test I2C simple sur 0x29");
    Wire.beginTransmission(0x29);
    byte error = Wire.endTransmission();
    if (error != 0) {
        debugPrintf(DBG_SENSORS, "ERREUR: LIDAR 0x29 non joignable, error=%d", error);
        return false;
    }

    debugPrintf(DBG_SENSORS, "Avant setTimeout");
    capteur.setTimeout(500);

    debugPrintf(DBG_SENSORS, "Avant capteur.init()");
    bool ok = capteur.init();
    debugPrintf(DBG_SENSORS, "Après capteur.init(), ok=%d", ok);
    if (!ok) {
        debugPrintf(DBG_SENSORS, "ERREUR: capteur.init() a échoué");
        return false;
    }

    debugPrintf(DBG_SENSORS, "Avant startContinuous");
    capteur.startContinuous(50);
    debugPrintf(DBG_SENSORS, "Après startContinuous");

    debugPrintf(DBG_SENSORS, "LIDAR (Pololu) initialisé en continu");
    return true;
}



// ===============================
// Lecture de la distance
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
