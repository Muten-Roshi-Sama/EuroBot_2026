#pragma once
#include "Task.h"
#include "Movement.h"

/*
    Two simple concrete tasks:
    - MoveTask(distanceCm)
    - RotateTask(angleDeg)

    Each implements:
    start(), update(), handleInterrupt(), resume(), cancel()
*/



class MoveTask : public Task {
    public:
        MoveTask(float distanceCm_, uint8_t speed = 0, unsigned long timeoutMs = 0)
            : Task(speed, timeoutMs), distanceCm(distanceCm_) {}

    void start(Movement &mv) override;
    void update(Movement &mv) override;
    TaskInterruptAction handleInterrupt(Movement &mv, uint8_t isrFlags) override;
    void resume(Movement &mv) override;
    void cancel(Movement &mv) override;

    private:
        float distanceCm = 0.0f;
        long targetTicks = 0;

        // PID variables
        float integralError = 0.0f;
        int loopCounter = 0;
        unsigned long lastPidLoopMs = 0;

        // Tunables / runtime variables
        int baseSpeed = 0;
        int minSpeed = 0;
        int maxSpeed = 255;
        int warmupIterations = 0;
        float Kp = 0.0f;
        float Ki = 0.0f;
        float deadZone = 0.0f;
};

class RotateTask : public Task {
    public:
        RotateTask(float angleDeg_, uint8_t speed = 0, unsigned long timeoutMs = 0)
        : Task(speed, timeoutMs), angleDeg(angleDeg_) {}

        void start(Movement &mv) override;
        void update(Movement &mv) override;
        TaskInterruptAction handleInterrupt(Movement &mv, uint8_t isrFlags) override;
        void resume(Movement &mv) override;
        void cancel(Movement &mv) override;

    private:
        float angleDeg = 0.0f;

        // PID/internal state
        float integralError = 0.0f;
        int loopCounter = 0;
        unsigned long lastPidLoopMs = 0;

        // Tunables
        float Kp = 1.3f;
        float Ki = 0.08f;
        int minSpeed = 50;
        int maxSpeed = 90;
        float deadZone = 3.5f;
};






