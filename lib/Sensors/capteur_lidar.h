#ifndef CAPTEUR_LIDAR_H
#define CAPTEUR_LIDAR_H

#include <Arduino.h>

// Initialise le capteur LiDAR
bool initLidar();

// Lit la distance en millimètres
int lireDistance();

// Affiche l'état de l'obstacle selon la distance
void afficherEtatObstacle(int distance);

#endif