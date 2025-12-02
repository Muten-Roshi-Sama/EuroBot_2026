#pragma once
#include "Task.h"
#include "Movement.h" // Important pour connaître la classe Movement

class MoveTask : public Task {
public:
    enum class MoveTaskMode { MOVE_DISTANCE, ROTATE_ANGLE };

    MoveTask(MoveTaskMode mode, float value, uint8_t speed = 0, unsigned long timeoutMs = 0)
        : Task(speed, timeoutMs), mode(mode), value(value) {}

    void start(Movement &mv) override;
    void update(Movement &mv) override;
    TaskInterruptAction handleInterrupt(Movement &mv, uint8_t isrFlags) override;
    
    void resume(Movement &mv);
    void cancel(Movement &mv) override;

private:
    MoveTaskMode mode;
    float value;      // Distance en cm ou Angle en degrés
    long targetTicks; // Cible convertie en ticks

    // --- PI CONTROL VARIABLES (Mémoire) ---
    float persistentError;
    float integral;
    int loopCounter;
    unsigned long lastPidLoopMs; // Timer pour remplacer le delay()

    // --- PARAMÈTRES PI (Identiques à ta fonction moveDistance) ---
    // Tu peux les déplacer dans settings.h si tu préfères
    const float Kp = 0.5f;
    const float Ki = 0.6f;
    const float integralMax = 300.0f;
    const float deadzone = 2.0f;
    const int warmupIterations = 50;
};