#pragma once

#include <Arduino.h>

// Pins
#define LIDAR_PIN 4  // Adapter selon ton câblage XSHUT/INT

// Flags ISR
#define ISR_FLAG_OBSTACLE         0x02
#define ISR_FLAG_OBSTACLE_CLEARED 0x04

// Fonctions publiques
bool initLidar();
int lireDistanceLidar1();
int lireDistanceLidar2();
int lireDistanceLidar3();
void afficherEtatObstacle(int distance);
void lidarSetupPin();
