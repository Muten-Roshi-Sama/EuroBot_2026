#pragma once
#include <Arduino.h>

struct Pose {
    float x;     // en mm
    float y;     // en mm
    float theta; // en radians
};

void odomInit();
void odomUpdate(long leftTicks, long rightTicks);
Pose odomGetPose();
void odomReset(float x = 0, float y = 0, float theta = 0);
