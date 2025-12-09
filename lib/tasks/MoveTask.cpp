#include "MoveTask.h"
#include "settings.h"
#include "isr_flags.h"
#include "../util/Debug.h"
#include <Arduino.h> 

// Gyro imu;
// SimplePID headingPid;


// Helper: absolute long pour les ticks
static inline long labs_long(long v) { return v < 0 ? -v : v; }


// ============ MOVE TASK ==================

void MoveTask::start(Movement &mv) {
    started = true;
    startMs = millis();
    paused = false;
    finished = false;
    cancelled = false;

    mv.resetEncoders();

    // PID/internal init
    integralError = 0.0f;
    loopCounter = 0;
    lastPidLoopMs = millis();

    // Tunables / defaults for distance
    Kp = 0.3f;
    Ki = 0.1f;
    warmupIterations = 30;
    baseSpeed = getSpeed() ? getSpeed() : mv.defaultSpeed;
    minSpeed = 85;
    maxSpeed = 255;
    deadZone = 0.0f;

    targetTicks = labs_long(mv.cmToTicks(distanceCm));

    // Start moving slowly; update() will ramp and sync wheels
    if (distanceCm >= 0.0f) mv.forward(minSpeed); else mv.backward(minSpeed);

    debugPrintf(DBG_MOVEMENT, "MoveTask START dist=%.1fcm tgt=%ld base=%d min=%d Kp=%.2f",
            distanceCm, targetTicks, baseSpeed, minSpeed, Kp);
}

void MoveTask::update(Movement &mv) {
    if (cancelled || finished || paused) return;

    if (timeoutMs && (millis() - startMs) > timeoutMs) {
        mv.stop();
        cancelled = true;
        finished = true;
        debugPrintf(DBG_MOVEMENT, "MoveTask TIMEOUT after %lums", millis() - startMs);
        return;
    }

    if (millis() - lastPidLoopMs < MOVEMENT_LOOP_DELAY) return;
    float dt = (millis() - lastPidLoopMs) / 1000.0f;
    lastPidLoopMs = millis();

    long leftTicks, rightTicks;
    noInterrupts();
    leftTicks = mv.getLeftTicks();
    rightTicks = mv.getRightTicks();
    interrupts();

    // debugPrintf(DBG_MOVEMENT, "MoveTask %ld/%ld \n", leftTicks, rightTicks);


    long progressTicks = max(labs_long(leftTicks), labs_long(rightTicks));
    if (progressTicks >= targetTicks) {
        mv.stop();
        finished = true;
        debugPrintf(DBG_MOVEMENT, "MoveTask DONE %ld/%ld loops=%d", progressTicks, targetTicks, loopCounter);
        return;
    }
    

    // Sync error (left - right)
    float error = (float)(leftTicks - rightTicks);
    integralError += error * dt;
    float correction = Kp * error + Ki * integralError;
    float corrPWM = correction;

    // Warmup ramp for base speed
    float rampFactor = 1.0f;
    // if (loopCounter < warmupIterations && warmupIterations > 0) {
    //     rampFactor = (float)loopCounter / (float)warmupIterations;
    //     if (rampFactor < 0.05f) rampFactor = 0.05f;
    // }
    int targetBase = minSpeed + (int)((baseSpeed - minSpeed) * rampFactor);

    int leftPWM = (int)constrain((float)targetBase - corrPWM, (float)minSpeed, 255.0f);
    int rightPWM = (int)constrain((float)targetBase + corrPWM, (float)minSpeed, 255.0f);
    debugPrintf(DBG_MOVEMENT, " err=%.1f corr=%.1f L=%d R=%d",
            int(error), int(correction), leftTicks, rightTicks);

    if (distanceCm >= 0.0f) {
        mv.motorLeft->setSpeed(leftPWM);
        mv.motorRight->setSpeed(rightPWM);
        mv.motorLeft->run(FORWARD);
        mv.motorRight->run(FORWARD);
    } else {
        mv.motorLeft->setSpeed(leftPWM);
        mv.motorRight->setSpeed(rightPWM);
        mv.motorLeft->run(BACKWARD);
        mv.motorRight->run(BACKWARD);
    }

    loopCounter++;
    mv.updateEncoderTimestamps();
    }

TaskInterruptAction MoveTask::handleInterrupt(Movement &mv, uint8_t isrFlags) {
    debugPrintf(DBG_MOVEMENT, "MoveTask ISR flags=0x%02X", isrFlags);
    if (isrFlags & ISR_FLAG_EMERGENCY) {
        mv.stop();
        cancelled = true;
        finished = true;
        debugPrintf(DBG_MOVEMENT, "MoveTask ISR: EMERGENCY -> CANCEL");
        return TaskInterruptAction::CANCEL;
    }
    if (isrFlags & ISR_FLAG_OBSTACLE) {
        mv.stop();
        paused = true;
        debugPrintf(DBG_MOVEMENT, "MoveTask ISR: OBSTACLE -> PAUSE");
        return TaskInterruptAction::PAUSE;
    }
    if (isrFlags & ISR_FLAG_OBSTACLE_CLEARED && paused) {
        paused = false;
        lastPidLoopMs = millis();
        debugPrintf(DBG_MOVEMENT, "MoveTask ISR: OBSTACLE_CLEARED -> RESUME");
        return TaskInterruptAction::HANDLE;
    }
    return TaskInterruptAction::IGNORE;
}

void MoveTask::cancel(Movement &mv) {
    cancelled = true;
    mv.stop();
    finished = true;
    debugPrintf(DBG_MOVEMENT, "MoveTask CANCELLED");
}

void MoveTask::resume(Movement &mv) {
    if (!paused || cancelled || finished) {
        debugPrintf(DBG_MOVEMENT, "MoveTask resume ignored paused=%d cancelled=%d finished=%d", paused, cancelled, finished);
        return;
    }
    paused = false;
    lastPidLoopMs = millis();
    debugPrintf(DBG_MOVEMENT, "MoveTask RESUMED");
}


// ============ ROTATE TASK ==================
void RotateTask::start(Movement &mv) {
    started = true;
    startMs = millis();
    paused = false;
    finished = false;
    cancelled = false;

    mv.resetEncoders();

    integralError = 0.0f;
    loopCounter = 0;
    lastPidLoopMs = millis();

    // Rotation tunables
    Kp = 1.3f;
    Ki = 0.08f;
    minSpeed = 50;
    maxSpeed = 90;
    deadZone = 3.5f;

    debugPrintf(DBG_MOVEMENT, "RotateTask START deg=%.1f Kp=%.2f Ki=%.2f dead=%0.1f", angleDeg, Kp, Ki, deadZone);
}

void RotateTask::update(Movement &mv) {
    if (cancelled || finished || paused) return;

    if (timeoutMs && (millis() - startMs) > timeoutMs) {
        mv.stop();
        cancelled = true;
        finished = true;
        // debugPrintf(DBG_MOVEMENT, "RotateTask TIMEOUT after %lums", millis() - startMs);
        return;
    }

    if (millis() - lastPidLoopMs < MOVEMENT_LOOP_DELAY) return;
    float dt = (millis() - lastPidLoopMs) / 1000.0f;
    lastPidLoopMs = millis();

    long leftTicks, rightTicks;
    noInterrupts();
    leftTicks = mv.getLeftTicks();
    rightTicks = mv.getRightTicks();
    interrupts();

    long avgTicks = (labs_long(leftTicks) + labs_long(rightTicks)) / 2;
    float currentAngle = mv.ticksToDegrees(avgTicks);

    float targetAngleAbs = abs(angleDeg);
    float error = targetAngleAbs - currentAngle;

    if (abs(error) <= deadZone) {
        mv.stop();
        finished = true;
        debugPrintf(DBG_MOVEMENT, "RotateTask DONE ang=%.2f tgt=%.1f", currentAngle, targetAngleAbs);
        return;
    }

    integralError += error * dt;
    // basic anti-windup
    if (integralError > 50.0f) integralError = 50.0f;
    if (integralError < -50.0f) integralError = -50.0f;

    float pidOutput = (Kp * error) + (Ki * integralError);
    int speed = (int)constrain(abs(pidOutput), (float)minSpeed, (float)maxSpeed);

    bool originalDirectionIsRight = (angleDeg >= 0.0f);
    bool shouldGoOriginalWay = (error > 0.0f);
    bool goRight = (originalDirectionIsRight == shouldGoOriginalWay);

    if (goRight) {
        mv.motorLeft->setSpeed(speed);
        mv.motorRight->setSpeed(speed);
        mv.motorLeft->run(FORWARD);
        mv.motorRight->run(BACKWARD);
    } else {
        mv.motorLeft->setSpeed(speed);
        mv.motorRight->setSpeed(speed);
        mv.motorLeft->run(BACKWARD);
        mv.motorRight->run(FORWARD);
    }

    debugPrintf(DBG_MOVEMENT, "ROT U: ang=%.1f cur=%.2f err=%.2f pid=%.2f spd=%d dir=%s",
                targetAngleAbs, currentAngle, error, pidOutput, speed, goRight ? "R" : "L");

    loopCounter++;
    mv.updateEncoderTimestamps();
}

TaskInterruptAction RotateTask::handleInterrupt(Movement &mv, uint8_t isrFlags) {
    if (isrFlags & ISR_FLAG_EMERGENCY) {
        mv.stop();
        cancelled = true;
        finished = true;
        debugPrintf(DBG_MOVEMENT, "RotateTask ISR: EMERGENCY -> CANCEL");
        return TaskInterruptAction::CANCEL;
    }
    if (isrFlags & ISR_FLAG_OBSTACLE) {
        mv.stop();
        paused = true;
        debugPrintf(DBG_MOVEMENT, "RotateTask ISR: OBSTACLE -> PAUSE");
        return TaskInterruptAction::PAUSE;
    }
    if (isrFlags & ISR_FLAG_OBSTACLE_CLEARED && paused) {
        paused = false;
        lastPidLoopMs = millis();
        debugPrintf(DBG_MOVEMENT, "RotateTask ISR: OBSTACLE_CLEARED -> RESUME");
        return TaskInterruptAction::HANDLE;
    }
    return TaskInterruptAction::IGNORE;
}

void RotateTask::cancel(Movement &mv) {
    cancelled = true;
    mv.stop();
    finished = true;
    debugPrintf(DBG_MOVEMENT, "RotateTask CANCELLED");
}

void RotateTask::resume(Movement &mv) {
        if (!paused || cancelled || finished) return;
        paused = false;
        lastPidLoopMs = millis();
        debugPrintf(DBG_MOVEMENT, "RotateTask RESUMED");
}

//             // Reset variables
//             gyro_z_cal_sum = 0;
//             angle_z = 0;
//             integral = 0;
//             lastError = 0;
            
//             debugPrintf(DBG_MOVEMENT, "MPU6050 Waking up...");
//             delay(50); // Petite pause pour laisser le capteur s'allumer
//         }

//         // Lecture pour calibration
//         Wire.beginTransmission(0x68);
//         Wire.write(0x47); 
//         Wire.endTransmission();
//         Wire.requestFrom(0x68, 2);
        
//         if (Wire.available() >= 2) {
//             int16_t rawZ = Wire.read() << 8 | Wire.read();
//             gyro_z_cal_sum += rawZ;
//             calibration_count++;
//         }

//         // FIN DE CALIBRATION
//         if (calibration_count >= CALIBRATION_SAMPLES) {
//             gyro_z_cal = (float)gyro_z_cal_sum / (float)CALIBRATION_SAMPLES;
//             previousTime = micros(); 
//             subState = STATE_MOVING; 
            
//             // Moteurs ON
//             int startSpeed = 150; 
//             if (value >= 0) { 
//                 mv.motorLeft->run(FORWARD); mv.motorRight->run(FORWARD);
//             } else {
//                 mv.motorLeft->run(BACKWARD); mv.motorRight->run(BACKWARD);
//             }
//             mv.motorLeft->setSpeed(startSpeed);
//             mv.motorRight->setSpeed(startSpeed);
            
//             debugPrintf(DBG_MOVEMENT, "Calibration Done. Offset: %d", (int)gyro_z_cal);
//         }
//         return; 
//     }












// =====================================

