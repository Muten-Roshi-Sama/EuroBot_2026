#pragma once
#include <Arduino.h>
#include <L298NX2.h>          // AndreaLombardo L298N library
#include "globals.h"        // contains PID, SensorsData, RobotCommand etc
#include "config.h"         // pin definitions and physical constants


/*
    L298N2X.h :
        - wrapper for ocntrolling 2 motors
        - we are only interested in methods compatible with our PID control.
        What not to use : 
            - forwardFor, backwardFor, runFor (with callback or not) : time-based blocking-style helpers.
            - fn that controls both motors at the same time.

        - use built-in callback function as motor safetystop ? void safetyStop() {motors.stop();} AND motors.forwardFor(5000, safetyStop);

*/


class Movement {
public:
    Movement(uint8_t ena, uint8_t in1, uint8_t in2,
            uint8_t enb, uint8_t in3, uint8_t in4);

    void startForward(float distanceCm);
    void startRotate(float angleDeg);

    void update(const SensorsData &sensors);
    bool isDone() const;

    void setDistancePID(float kp, float ki, float kd);
    void setAnglePID(float kp, float ki, float kd);

    void stopMotors();


private:
    L298NX2 motors;

    PID pidDistance;
    PID pidAngle;

    MovementTarget target;

    unsigned long lastUpdateUs;
    float startDistance, startYaw;

    void resetPID(PID &pid);
    float computePID(PID &pid, float error, float dt, 
            float &pTerm, float &iTerm, float &dTerm);

    void applyMotorOutputs(float leftCmd, float rightCmd);
    
};

