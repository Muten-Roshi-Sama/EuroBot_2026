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
    // À ajouter dans ta classe / header
    enum SubState { STATE_CALIBRATION, STATE_MOVING };
    SubState subState = STATE_CALIBRATION;

    long gyro_z_cal_sum = 0;
    int calibration_count = 0;
    float gyro_z_cal = 0;
    float angle_z = 0;
    float integral = 0;
    float lastError = 0;
    unsigned long previousTime = 0;

    // Constantes PID (peuvent être static ou #define)
    const float Kp_gyro = 0.8f;
    const float Ki_gyro = 0.02f;
    const float Kd_gyro = 0.5f;
    const int CALIBRATION_SAMPLES = 50; // Réduit pour ne pas attendre 10 ans, ou garde 2000 si ta boucle est très rapide
};