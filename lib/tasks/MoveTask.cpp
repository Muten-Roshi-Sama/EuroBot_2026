#include "MoveTask.h"
#include "settings.h"
#include "isr_flags.h"
#include "../util/Debug.h"
#include <Arduino.h> 

// Helper: absolute long pour les ticks
static inline long labs_long(long v) { return v < 0 ? -v : v; }

void MoveTask::start(Movement &mv) {
    started = true;
    startMs = millis();
    paused = false;
    finished = false;
    cancelled = false;

    mv.resetEncoders();

    integralError = 0.0f;
    loopCounter = 0;
    lastPidLoopMs = millis();

    // --- CONFIGURATION SELON LE MODE ---
    if (mode == MoveTaskMode::MOVE_DISTANCE) {
        Kp = 0.8f; 
        Ki = 0.0f;
        warmupIterations = 30;
        baseSpeed = getSpeed() ? getSpeed() : mv.defaultSpeed;
        minSpeed = 110;
        maxSpeed = 255;
        deadZone = 0.0f; 

        targetTicks = labs_long(mv.cmToTicks(value));
        
        if (value >= 0.0f) mv.forward(minSpeed); else mv.backward(minSpeed);
        debugPrintf(DBG_MOVEMENT, "Start DIST: Val=%.1fcm Ticks=%ld", value, targetTicks);

    } else {
        // --- ROTATION ---
        Kp = 1.3f; 
        Ki = 0.08f;
        warmupIterations = 0; 
        baseSpeed = 0; 
        minSpeed = 50;  
        maxSpeed = 90;  
        deadZone = 3.5f; 

        debugPrintf(DBG_MOVEMENT, "Start ROT: Val=%.1fdeg", value);
    }
}

void MoveTask::update(Movement &mv) {
    if (cancelled || finished || paused) return;

    if (timeoutMs && (millis() - startMs) > timeoutMs) {
        mv.stop(); cancelled = true; finished = true; return;
    }

    if (millis() - lastPidLoopMs < MOVEMENT_LOOP_DELAY) return; 
    float dt = (millis() - lastPidLoopMs) / 1000.0f; 
    lastPidLoopMs = millis(); 

    long leftTicks, rightTicks;
    noInterrupts();
    leftTicks  = mv.getLeftTicks();
    rightTicks = mv.getRightTicks();
    interrupts();

    // ================= DISTANCE =================
    if (mode == MoveTaskMode::MOVE_DISTANCE) {
        long progressTicks = max(labs_long(leftTicks), labs_long(rightTicks));
        
        if (progressTicks >= targetTicks) {
            mv.stop(); finished = true; return;
        }

        float error = (float)(leftTicks - rightTicks);
        integralError += error * Ki; 
        float correction = Kp * error + integralError;
        float corrPWM = correction; 

        float rampFactor = 1.0f;
        if (loopCounter < warmupIterations) {
            rampFactor = (float)loopCounter / (float)warmupIterations;
            if (rampFactor < 0.05f) rampFactor = 0.05f;
        }
        int targetBase = minSpeed + (int)((baseSpeed - minSpeed) * rampFactor);

        int leftPWM = (int)constrain((float)targetBase - corrPWM, (float)minSpeed, 255.0f);
        int rightPWM = (int)constrain((float)targetBase + corrPWM, (float)minSpeed, 255.0f);

        if (value >= 0.0f) {
            mv.motorLeft->setSpeed(leftPWM); mv.motorRight->setSpeed(rightPWM);
            mv.motorLeft->run(FORWARD);      mv.motorRight->run(FORWARD);
        } else {
            mv.motorLeft->setSpeed(leftPWM); mv.motorRight->setSpeed(rightPWM);
            mv.motorLeft->run(BACKWARD);     mv.motorRight->run(BACKWARD);
        }
    } 
    // ================= ROTATION =================
    else { 
        long avgTicks = (labs_long(leftTicks) + labs_long(rightTicks)) / 2;
        float currentAngle = mv.ticksToDegrees(avgTicks); 

        float targetAngleAbs = fabs(value); 
        float error = targetAngleAbs - currentAngle;

        if (fabs(error) <= deadZone) {
            mv.stop(); finished = true; return;
        }

        integralError += error * dt; 
        if (integralError > 50.0f) integralError = 50.0f;
        if (integralError < -50.0f) integralError = -50.0f;

        float pidOutput = (Kp * error) + (Ki * integralError);
        int speed = (int)constrain(fabs(pidOutput), (float)minSpeed, (float)maxSpeed);

        bool goRight = ((value >= 0) == (error > 0));

        if (goRight) {
            mv.motorLeft->setSpeed(speed);  mv.motorRight->setSpeed(speed);
            mv.motorLeft->run(FORWARD);     mv.motorRight->run(BACKWARD);
        } else {
            mv.motorLeft->setSpeed(speed);  mv.motorRight->setSpeed(speed);
            mv.motorLeft->run(BACKWARD);    mv.motorRight->run(FORWARD);
        }
    }
    loopCounter++;
    mv.updateEncoderTimestamps();
}

TaskInterruptAction MoveTask::handleInterrupt(Movement &mv, uint8_t isrFlags) {
    if (isrFlags & ISR_FLAG_EMERGENCY) {
        mv.stop(); cancelled = true; finished = true; return TaskInterruptAction::CANCEL;
    }
    if (isrFlags & ISR_FLAG_OBSTACLE) {
        mv.stop(); paused = true; return TaskInterruptAction::PAUSE;
    }
    if (isrFlags & ISR_FLAG_OBSTACLE_CLEARED && paused) {
        paused = false; lastPidLoopMs = millis(); return TaskInterruptAction::HANDLE;
    }
    return TaskInterruptAction::IGNORE;
}

void MoveTask::cancel(Movement &mv) { cancelled = true; mv.stop(); finished = true; }
void MoveTask::resume(Movement &mv) { if (!paused || cancelled || finished) return; paused = false; lastPidLoopMs = millis(); }