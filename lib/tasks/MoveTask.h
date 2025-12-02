#pragma once
#include "Task.h"
#include "Movement.h"

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
    float value;      
    long targetTicks; 

    // Variables internes PID
    float integralError;
    int loopCounter;
    unsigned long lastPidLoopMs;
    
    // Paramètres qui changent selon le mode (Distance ou Rotation)
    int baseSpeed;
    int minSpeed;
    int maxSpeed; // Ajout pour limiter la rotation (90 dans ton cas)
    int warmupIterations;
    float Kp;
    float Ki;
    float deadZone; // Ajout pour la rotation (3.5°)
};