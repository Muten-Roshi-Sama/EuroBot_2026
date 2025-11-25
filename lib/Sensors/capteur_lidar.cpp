#include "capteur_lidar.h"
#include <Wire.h>
#include <VL53L0X.h>

VL53L0X capteur;


bool initLidar() {
    Wire.begin();

    if (!capteur.init()) {
        return false;
    }

    capteur.setTimeout(500);
    capteur.startContinuous();

    return true;
}

int lireDistance() {
    int distance = capteur.readRangeContinuousMillimeters();

    if (capteur.timeoutOccurred()) {
        return -1; // Erreur
    }

    if (distance > 8190) {
        return -2; // Hors portée
    }

    return distance;
}

void afficherEtatObstacle(int distance) {
    if (distance == -1) {
        Serial.println("ERREUR - Pas de mesure");
        return;
    }

    if (distance == -2) {
        Serial.println("---- mm | Hors de portee");
        return;
    }

    // Formatage de l'affichage
    if (distance < 1000) Serial.print(" ");
    if (distance < 100) Serial.print(" ");
    if (distance < 10) Serial.print(" ");

    Serial.print(distance);
    Serial.print(" mm | ");

    // Classification
    if (distance < 200) {
        Serial.println("ALERTE! OBSTACLE TRES PROCHE!");
    } else if (distance < 500) {
        Serial.println("Obstacle proche");
    } else if (distance < 1000) {
        Serial.println("Obstacle a distance moyenne");
    } else {
        Serial.println("Voie libre");
    }
}