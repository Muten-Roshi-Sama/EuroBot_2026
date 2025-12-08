#include "odometry.h"
#include "settings.h"
#include <math.h>

// Conversion de cm → mm
#define WHEEL_DIAMETER_MM (WHEEL_DIAMETER * 10.0f)
#define WHEEL_BASE_MM     (WHEEL_BASE * 10.0f)

static Pose pose;

// Pour stocker les anciens ticks
static long prevLeftTicks = 0;
static long prevRightTicks = 0;

void odomInit() {
    pose.x = 0;
    pose.y = 0;
    pose.theta = 0;

    prevLeftTicks = 0;
    prevRightTicks = 0;
}

void odomReset(float x, float y, float theta) {
    pose.x = x;
    pose.y = y;
    pose.theta = theta;

    prevLeftTicks = 0;
    prevRightTicks = 0;
}

Pose odomGetPose() {
    return pose;
}

void odomUpdate(long leftTicks, long rightTicks) {
    // Δ ticks
    long dL = leftTicks - prevLeftTicks;
    long dR = rightTicks - prevRightTicks;

    prevLeftTicks = leftTicks;
    prevRightTicks = rightTicks;

    // Tick -> distance roue
    float distancePerTick = (PI * WHEEL_DIAMETER_MM) / ENCODER_RESOLUTION;

    float distL = dL * distancePerTick;
    float distR = dR * distancePerTick;

    // Mouvement du robot
    float dS = (distR + distL) * 0.5f;                    // déplacement en mm
    float dTheta = (distR - distL) / WHEEL_BASE_MM;      // rotation en rad

    // Mise à jour de la position
    pose.x += dS * cos(pose.theta + dTheta * 0.5f);
    pose.y += dS * sin(pose.theta + dTheta * 0.5f);
    pose.theta += dTheta;

    // Normalisation
    if (pose.theta > PI) pose.theta -= 2 * PI;
    if (pose.theta < -PI) pose.theta += 2 * PI;
}
