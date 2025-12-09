



#include "GyroMoveTask.h"
#include "../util/Debug.h"
#include <Wire.h>
#include <Arduino.h>
#include "isr_flags.h"

// MPU-6050 default I2C addr and registers
#define MPU_ADDR 0x68
#define MPU_GYRO_Z_H 0x47
#define MPU_PWR_MGMT_1 0x6B
// Sensitivity for ±250 dps full scale (LSB per deg/s)
#define MPU_SENS_250DPS 131.0f

// ---------- Constructors ----------
GyroMoveTask::GyroMoveTask(float distanceCm, int speed, float estCmPerSec) {
    if (estCmPerSec <= 0.01f) estCmPerSec = 10.0f;
    durationMs = (unsigned long)(fabs(distanceCm) / estCmPerSec * 1000.0f);
    baseSpeed = speed;
    forward = (distanceCm >= 0.0f);
}

GyroMoveTask::GyroMoveTask(unsigned long durationMs_, int speed) {
    durationMs = durationMs_;
    baseSpeed = speed;
    forward = true;
}

// ---------- IMU helpers ----------
void GyroMoveTask::imuBegin() {
    Wire.begin();
    // Wake up MPU
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(MPU_PWR_MGMT_1);
    Wire.write(0x00);
    Wire.endTransmission();
    delay(5);
    // initialize integrator/time
    lastMicros = micros();
    headingDeg = 0.0f;
}

int16_t GyroMoveTask::readRawGyroZ() {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(MPU_GYRO_Z_H);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, (uint8_t)2, (uint8_t)true);
    if (Wire.available() < 2) return 0;
    int16_t hi = Wire.read();
    int16_t lo = Wire.read();
    return (int16_t)((hi << 8) | (uint8_t)lo);
}

// ---------- Task lifecycle ----------
void GyroMoveTask::start(Movement &mv) {
    started = true;
    startMs = millis();
    lastMicros = micros();
    paused = false;
    finished = false;
    cancelled = false;

    // Ensure motors are stopped during gyro calibration
    mv.stop();
    delay(50);

    // IMU init + quick calibration (stationary)
    imuBegin();
    const int samples = 200;
    long sum = 0;
    for (int i = 0; i < samples; ++i) {
        int16_t gz = readRawGyroZ();
        sum += gz;
        delay(2);
    }
    float avgRaw = (float)sum / (float)samples;
    gyroBiasDegPerSec = avgRaw / MPU_SENS_250DPS;

    // target heading = current integrated heading (0 after imuBegin)
    targetHeading = headingDeg;

    // reset PID state
    integ = 0.0f;
    lastErr = 0.0f;

    // Start motors at base speed
    int sp = constrain(baseSpeed, 0, 255);
    if (forward) {
        mv.motorLeft->setSpeed(sp);
        mv.motorRight->setSpeed(sp);
        mv.motorLeft->run(FORWARD);
        mv.motorRight->run(FORWARD);
    } else {
        mv.motorLeft->setSpeed(sp);
        mv.motorRight->setSpeed(sp);
        mv.motorLeft->run(BACKWARD);
        mv.motorRight->run(BACKWARD);
    }

    debugPrintf(DBG_MOVEMENT, "GyroMove START dur=%lums sp=%d bias=%.2f", durationMs, baseSpeed, gyroBiasDegPerSec);
}

void GyroMoveTask::update(Movement &mv) {
    if (cancelled || finished || paused) return;

    // check timeout by duration
    if (durationMs > 0 && (millis() - startMs) >= durationMs) {
        mv.stop();
        finished = true;
        debugPrintf(DBG_MOVEMENT, "GyroMove DONE after %lums", millis() - startMs);
        return;
    }

    // integrate gyro and compute PID
    unsigned long nowMicros = micros();
    float dt = (nowMicros - lastMicros) / 1e6f;
    if (dt <= 0.0f) dt = 0.001f;
    lastMicros = nowMicros;

    int16_t rawGz = readRawGyroZ();
    float gz = ((float)rawGz) / MPU_SENS_250DPS; // deg/s raw -> deg/s
    float gzCorr = gz - gyroBiasDegPerSec;
    headingDeg += gzCorr * dt;
    // normalize to -180..180
    if (headingDeg > 180.0f) headingDeg -= 360.0f;
    if (headingDeg <= -180.0f) headingDeg += 360.0f;

    // heading error = target - current, normalized
    float err = targetHeading - headingDeg;
    if (err > 180.0f) err -= 360.0f;
    if (err < -180.0f) err += 360.0f;

    // PID
    integ += err * dt;
    float deriv = (err - lastErr) / dt;
    float out = (Kp * err) + (Ki * integ) + (Kd * deriv);
    lastErr = err;
    // convert PID output (degrees) to PWM correction directly (tune Kp/Ki/Kd)
    float corrPWM = constrain(out, -maxCorrection, maxCorrection);

    int leftPWM = (int)constrain((float)baseSpeed - corrPWM, 0.0f, 255.0f);
    int rightPWM = (int)constrain((float)baseSpeed + corrPWM, 0.0f, 255.0f);

    if (forward) {
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

    // debugPrintf(DBG_MOVEMENT, "GYRO U dt=%.3f heading=%.2f err=%.2f corr=%.2f L=%d R=%d",
    //             dt, headingDeg, err, corrPWM, leftPWM, rightPWM);
}

TaskInterruptAction GyroMoveTask::handleInterrupt(Movement &mv, uint8_t isrFlags) {
    debugPrintf(DBG_MOVEMENT, "GyroMove ISR flags=0x%02X", isrFlags);
    if (isrFlags & ISR_FLAG_EMERGENCY) {
        mv.stop();
        cancelled = true;
        finished = true;
        debugPrintf(DBG_MOVEMENT, "GyroMove ISR: EMERGENCY -> CANCEL");
        return TaskInterruptAction::CANCEL;
    }
    if (isrFlags & ISR_FLAG_OBSTACLE) {
        mv.stop();
        paused = true;
        debugPrintf(DBG_MOVEMENT, "GyroMove ISR: OBSTACLE -> PAUSE");
        return TaskInterruptAction::PAUSE;
    }
    if (isrFlags & ISR_FLAG_OBSTACLE_CLEARED && paused) {
        paused = false;
        lastMicros = micros();
        debugPrintf(DBG_MOVEMENT, "GyroMove ISR: OBSTACLE_CLEARED -> RESUME");
        return TaskInterruptAction::HANDLE;
    }
    return TaskInterruptAction::IGNORE;
}

void GyroMoveTask::cancel(Movement &mv) {
    cancelled = true;
    mv.stop();
    finished = true;
    debugPrintf(DBG_MOVEMENT, "GyroMove CANCELLED");
}

void GyroMoveTask::resume(Movement &mv) {
    if (!paused || cancelled || finished) return;
    paused = false;
    lastMicros = micros();
    debugPrintf(DBG_MOVEMENT, "GyroMove RESUMED");
}

