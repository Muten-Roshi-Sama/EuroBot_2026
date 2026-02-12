#include "movement.h"

#include <math.h>

#include "Debug.h"
#include "config.h"
#include "globals.h"



// Constructor
// -----------------------
Movement::Movement(uint8_t enaPin, uint8_t in1Pin, uint8_t in2Pin,
                    uint8_t enbPin, uint8_t in3Pin, uint8_t in4Pin)
    : 
    motors(enaPin, in1Pin, in2Pin, enbPin, in3Pin, in4Pin), // pins

    pidDistance(DISTANCE_PID_DEFAULT.kp, DISTANCE_PID_DEFAULT.ki, DISTANCE_PID_DEFAULT.kd), // default values from config.h
    pidAngle(ANGLE_PID_DEFAULT.kp, ANGLE_PID_DEFAULT.ki, ANGLE_PID_DEFAULT.kd)
{
    target.active = false;
    lastUpdateUs = micros();
}


// Start movement commands
// -----------------------
void Movement::startForward(float distanceCm) {
    target.distanceCm = distanceCm;
    target.angleDeg = 0.0f;
    target.active = true;

    startDistance = 0;
    startYaw = 0;

    resetPID(pidDistance);
    resetPID(pidAngle);
}

void Movement::startRotate(float angleDeg) {
    target.distanceCm = 0.0f;
    target.angleDeg = angleDeg;
    target.active = true;

    startDistance = 0;
    startYaw = 0;

    resetPID(pidDistance);
    resetPID(pidAngle);
}

// Called every FSM cycle
// -----------------------
void Movement::update(const SensorsData &sensors)
{
    if (!target.active) return;

    unsigned long now = micros();
    float dt = (now - lastUpdateUs) / 1e6f;
    if (dt <= 0.0005f) dt = 0.001f;
    lastUpdateUs = now;

    // --- Current state ---
    float currentDistance =
        (sensors.encoderLeft.distance_cm +
         sensors.encoderRight.distance_cm) * 0.5f;

    float currentYaw = sensors.mpu.yaw;

    // --- Errors ---
    float distanceError = target.distanceCm - currentDistance;
    float angleError    = target.angleDeg - currentYaw;

    // Normalize angle error (-180, 180)
    while (angleError > 180) angleError -= 360;
    while (angleError < -180) angleError += 360;

    // --- PID outputs ---
    float pDist, iDist, dDist, pAngle, iAngle, dAngle;
    float linearCmd  = computePID(pidDistance, distanceError, dt, pDist, iDist, dDist);
    float angularCmd = computePID(pidAngle, angleError, dt, pAngle, iAngle, dAngle);

    // --- Differential drive mixing ---
    float leftCmd  = linearCmd - angularCmd;
    float rightCmd = linearCmd + angularCmd;

    applyMotorOutputs(leftCmd, rightCmd);

    // --- Stop condition ---
    if (fabs(distanceError) < 0.5f &&
        fabs(angleError) < 2.0f)
    {
        stopMotors();
        target.active = false;
    }

    // --- Debug prints ---
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 1000)
    {
        lastPrint = millis();

        debugPrintf(DBG_MOVEMENT,
            "DE:%.2f P:%.2f I:%.2f D:%.2f | AE:%.2f P:%.2f I:%.2f D:%.2f | L:%d R:%d | dt:%.3f",
            distanceError, pDist, iDist, dDist,
            angleError, pAngle, iAngle, dAngle,
            (int)leftCmd, (int)rightCmd, dt
        );
    }
}

// -----------------------
bool Movement::isDone() const { return !target.active; }


// ----------------------------
void Movement::setDistancePID(float kp, float ki, float kd)
{ pidDistance.kp = kp; pidDistance.ki = ki; pidDistance.kd = kd; 
}

// ----------------------------
void Movement::setAnglePID(float kp, float ki, float kd)
{ pidAngle.kp = kp; pidAngle.ki = ki; pidAngle.kd = kd; 
}

// ----------------------------
void Movement::resetPID(PID &pid)
{ pid.integral = 0; pid.previousError = 0; 
}

// ----------------------------
float Movement::computePID(PID &pid, float error, float dt, float &pTerm, float &iTerm, float &dTerm)
{
    // Proportional
    pTerm = pid.kp * error;
    
    // Integral : Simple anti-windup
    pid.integral += error * dt;
    pid.integral = constrain(pid.integral, -100, 100);
    iTerm = pid.ki * pid.integral;

    // Derivative
    float derivative = (error - pid.previousError) / dt;
    dTerm = pid.kd * derivative;

    // Error
    pid.previousError = error;

    return pTerm + iTerm + dTerm;
}

// ----------------------------
void Movement::applyMotorOutputs(float leftCmd, float rightCmd)
{
    int leftPWM  = constrain(abs((int)leftCmd),  0, 255);
    int rightPWM = constrain(abs((int)rightCmd), 0, 255);

    motors.setSpeedA(leftPWM);
    motors.setSpeedB(rightPWM);

    if (leftCmd > 0)
        motors.forwardA();
    else if (leftCmd < 0)
        motors.backwardA();
    else
        motors.stopA();

    if (rightCmd > 0)
        motors.forwardB();
    else if (rightCmd < 0)
        motors.backwardB();
    else
        motors.stopB();
    
    // ---- Debug prints ----
    if (leftPWM == 255 || rightPWM == 255) { debugPrintF(DBG_MOVEMENT, F("PWM SATURATION")); }
}

// ----------------------------
void Movement::stopMotors() { motors.stop(); }



