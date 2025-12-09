
// Add File: lib/tasks/GyroMoveTask.h
#pragma once

#include "Task.h"
#include "../movement/Movement.h"
// #include "../settings.h"



class GyroMoveTask : public Task {
public:
    // Constructor using distance estimate (distanceCm > 0 moves forward)
    GyroMoveTask(float distanceCm, int speed = DEFAULT_SPEED, float estCmPerSec = 10.0f);
    // Alternate constructor: explicit duration in ms
    GyroMoveTask(unsigned long durationMs, int speed);

    void start(Movement &mv) override;
    void update(Movement &mv) override;
    TaskInterruptAction handleInterrupt(Movement &mv, uint8_t isrFlags) override;
    void cancel(Movement &mv) override;
    void resume(Movement &mv) override;

private:
    // motion parameters
    unsigned long durationMs = 0;
    int baseSpeed = DEFAULT_SPEED;
    bool forward = true;

    // heading control
    float targetHeading = 0.0f;

    // internal timers
    unsigned long startMs = 0;
    unsigned long lastMicros = 0;

    // gyro calibration / state
    float gyroBiasDegPerSec = 0.0f;
    float headingDeg = 0.0f; // integrated heading (deg), signed

    // simple PID state
    float Kp = 0.8f;
    float Ki = 0.01f;
    float Kd = 0.05f;
    float integ = 0.0f;
    float lastErr = 0.0f;
    float maxCorrection = 120.0f; // max PWM correction

    // helpers (defined in .cpp)
    void imuBegin();
    int16_t readRawGyroZ();
};

