#ifndef NAVIGATION_H
#define NAVIGATION_H

#include "odometry.h"
#include "Movement.h"

// Initialise le module navigation avec le robot
void navigationInit(Movement &m);

// Calcule l'angle à atteindre pour aller vers la cible
float computeTargetAngle(float x, float y, float x_target, float y_target);

// Calcule la distance à parcourir vers la cible
float computeDistance(float x, float y, float x_target, float y_target);

// Calcule la prochaine étape à faire pour atteindre la cible
void computeNextStep(float x, float y, float theta,
                     float x_target, float y_target,
                     float &distanceToMove, float &angleToTurn);

// Déplace le robot vers un point donné (sans obstacle)
void goToPoint(float x_target, float y_target);

#endif