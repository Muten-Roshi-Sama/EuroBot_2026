#include "capteur_lidar.h"  // notre module LIDAR perso
#include <Wire.h>           // communication I2C
#include <VL53L0X.h>        // librairie officielle capteur VL53L0X
#include "Debug.h"          // système de debug custom avec debugPrintf
#include "settings.h"       // pins et configurations

VL53L0X capteur;           // instance du capteur VL53L0X

// ===============================
// Configuration de la pin du LIDAR (XSHUT ou INT)
// ===============================
void lidarSetupPin() {
    pinMode(LIDAR_PIN, OUTPUT);   // configure la pin comme sortie
    digitalWrite(LIDAR_PIN, HIGH); // active le capteur
}

// ===============================
// Initialisation du LIDAR
// ===============================
bool initLidar() {
    lidarSetupPin();    // configure la pin avant de démarrer
    Wire.begin();       // démarre le bus I2C

    if (!capteur.init()) { // initialise le capteur
        debugPrintf(DBG_SENSORS, "Erreur LIDAR : init échoué");
        return false;      // si échec → on renvoie false
    }

    capteur.setTimeout(500);    // timeout maximal pour les mesures (ms)
    capteur.startContinuous();  // mode lecture continue (non bloquant)

    debugPrintf(DBG_SENSORS, "LIDAR OK");  // debug si tout va bien
    return true;
}

// ===============================
// Lecture de la distance
// ===============================
int lireDistance() {
    int d = capteur.readRangeContinuousMillimeters();  // récupère la dernière mesure en mm

    if (capteur.timeoutOccurred()) {   // vérifie si le capteur a timeout
        return -1;                     // -1 = erreur de mesure
    }

    if (d > 8190) {                    // valeur hors portée du capteur
        return -2;                     // -2 = hors portée
    }

    // Debug uniquement si obstacle très proche (≤5 cm)
    if (d > 0 && d <= 50) {
        debugPrintf(DBG_SENSORS, "⚠️ Obstacle très proche : %d mm", d);
    }

    return d;  // retourne la distance mesurée
}

// ===============================
// Affichage d'état pour debug
// ===============================
void afficherEtatObstacle(int distance) {
    if (distance == -1) {                    // erreur timeout
        debugPrintf(DBG_SENSORS, "ERREUR - Pas de mesure");
        return;
    }

    if (distance == -2) {                    // hors portée
        debugPrintf(DBG_SENSORS, "---- mm | Hors de portee");
        return;
    }

    // distance valide
    debugPrintf(DBG_SENSORS, "Voie libre (%d mm)", distance);
}
