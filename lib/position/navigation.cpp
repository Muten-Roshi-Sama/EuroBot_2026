#include "navigation.h"
#include <math.h>

static Movement *robotMovement = nullptr;

void navigationInit(Movement &m) {
    robotMovement = &m;
}

float computeTargetAngle(float x, float y, float x_target, float y_target) {
    return atan2(y_target - y, x_target - x); // angle en radians
}

float computeDistance(float x, float y, float x_target, float y_target) {
    float dx = x_target - x;
    float dy = y_target - y;
    return sqrt(dx*dx + dy*dy); // distance euclidienne
}

void computeNextStep(float x, float y, float theta,
                     float x_target, float y_target,
                     float &distanceToMove, float &angleToTurn) {
    float targetAngle = computeTargetAngle(x, y, x_target, y_target);
    distanceToMove = computeDistance(x, y, x_target, y_target);

    // Angle relatif à l'orientation actuelle
    angleToTurn = targetAngle - theta;

    // Normalisation entre -PI et PI
    if (angleToTurn > M_PI) angleToTurn -= 2*M_PI;
    if (angleToTurn < -M_PI) angleToTurn += 2*M_PI;
}

void goToPoint(float x_target, float y_target) {
    if (robotMovement == nullptr) return;

    Pose currentPose = odomGetPose();
    float distance, angle;

    computeNextStep(currentPose.x, currentPose.y, currentPose.theta,
                    x_target, y_target,
                    distance, angle);

    // Tourner d'abord
    robotMovement->rotate(angle * 180.0 / M_PI); // conversion en degrés
    // Avancer ensuite
    robotMovement->moveDistance(distance / 10.0f); // conversion mm -> cm si nécessaire
}
