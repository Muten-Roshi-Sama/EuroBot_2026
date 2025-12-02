#ifndef CAPTEUR_LIDAR_H
#define CAPTEUR_LIDAR_H

#include <Arduino.h>

// Initialise le capteur LiDAR (VL53L0X)
bool initLidar();

// Lit la distance en millimètres (non bloquant)
// Retourne :
//   - distance en mm
//   - -1 si timeout
//   - -2 si hors portée
int lireDistance();

// Affiche le statut en fonction de la distance
void afficherEtatObstacle(int distance);

#endif
