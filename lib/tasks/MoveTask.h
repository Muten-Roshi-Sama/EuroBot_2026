#pragma once
#include "Task.h"
#include "../movement/Movement.h"

enum class MoveTaskMode {
  MOVE_DISTANCE,
  ROTATE_ANGLE
};

class MoveTask : public Task {
    public:
        // On change le constructeur pour accepter le mode
        MoveTask(MoveTaskMode mode, float value, uint8_t speed = 0, unsigned long timeoutMs = 0)
            : Task(speed, timeoutMs), mode(mode), value(value) {}

        void start(Movement &mv) override;
        void update(Movement &mv) override;
        TaskInterruptAction handleInterrupt(Movement &mv, uint8_t isrFlags) override;
        
        void resume(Movement &mv);
        void cancel(Movement &mv) override;

    private:
        MoveTaskMode mode; // La variable manquante
        float value;       // La variable générique (cm ou degrés)
        long targetTicks; 

        // PID variables
        float integralError = 0.0f;
        int loopCounter = 0;
        unsigned long lastPidLoopMs = 0;
        
        int baseSpeed = 0;
        int minSpeed = 0;
        int maxSpeed = 255;
        int warmupIterations;
        float Kp = 0.0f;
        float Ki = 0.0f;
        float deadZone = 0;
};