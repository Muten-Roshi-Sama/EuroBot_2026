
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
class RotateGyroTask : public Task {
public:
    /**
     * @param targetAngleDeg : Angle à tourner (ex: 90.0 pour droite, -90.0 pour gauche)
     * @param speed : Vitesse maximale des moteurs (0-255)
     * @param tolerance : Précision requise en degrés pour s'arrêter (défaut 2.0°)
     * @param timeoutMs : Sécurité pour arrêter si le robot est bloqué (défaut 5000ms)
     */
    RotateGyroTask(float targetAngleDeg, int speed, float tolerance = 2.0f, unsigned long timeoutMs = 5000);

    void start(Movement &mv) override;
    void update(Movement &mv) override;
    TaskInterruptAction handleInterrupt(Movement &mv, uint8_t isrFlags) override;
    void cancel(Movement &mv) override;
    void resume(Movement &mv) override;
    
    // Ajouté car la classe mère Task peut le demander, sinon optionnel selon votre Task.h
    //bool isFinished() const override { return finished; }

private:
    // Motion parameters
    float targetAngle = 0.0f;
    float toleranceDeg = 2.0f;
    int baseSpeed = 150;     // Vitesse max
    int minSpeed = 55;       // Vitesse min pour vaincre les frottements (ajustez selon votre robot)
    unsigned long durationMs = 0;

    // Internal state
    bool started = false;
    bool finished = false;
    bool paused = false;
    bool cancelled = false;

    // Timers
    unsigned long startMs = 0;
    unsigned long lastMicros = 0;

    // Gyro state
    float gyroBiasDegPerSec = 0.0f;
    float headingDeg = 0.0f; // Position actuelle (relative au début de la rotation)

    // PID state (Ajusté pour la rotation)
    float Kp = 3.5f;   // Plus agressif pour tourner
    float Ki = 0.1f;
    float Kd = 0.5f;
    float integ = 0.0f;
    float lastErr = 0.0f;

    // Helpers
    void imuBegin();
    int16_t readRawGyroZ();
};
