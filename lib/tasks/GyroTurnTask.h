#pragma once
#include "Movement.h"
#pragma once

#include "Task.h"
#include "../movement/Movement.h"

class GyroTurnTask : public Task {
public:
    GyroTurnTask(float angleDeg, int maxSpeed = 80);

    void start(Movement &mv);
    void update(Movement &mv);
    void cancel(Movement &mv);

    bool isFinished() const { return finished; }

private:
    // IMU
    void imuBegin();
    int16_t readRawGyroZ();

    // params
    float targetHeading = 0.0f;
    float headingDeg = 0.0f;
    float gyroBiasDegPerSec = 0.0f;
    float gzFilt = 0.0f;

    // timing
    unsigned long lastMicros = 0;

    // PID
    float Kp = 0.8f;
    float Ki = 0.01f;
    float Kd = 0.05f;

    float integ = 0.0f;
    float lastErr = 0.0f;

    // motion
    float requestedAngle;
    int maxPWM;
    bool finished = false;
};
