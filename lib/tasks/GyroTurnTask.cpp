#include "GyroTurnTask.h"
#include <Wire.h>
#include <Arduino.h>
#include "L298N.h"

// MPU6050
#define MPU_ADDR 0x68
#define MPU_GYRO_Z_H 0x47
#define MPU_PWR_MGMT_1 0x6B
#define MPU_SENS_250DPS 131.0f

GyroTurnTask::GyroTurnTask(float angleDeg, int maxSpeed)
    : requestedAngle(angleDeg), maxPWM(maxSpeed) {}

void GyroTurnTask::imuBegin() {
    Wire.begin();
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(MPU_PWR_MGMT_1);
    Wire.write(0x00);
    Wire.endTransmission();
    delay(5);

    headingDeg = 0.0f;
    lastMicros = micros();
}

int16_t GyroTurnTask::readRawGyroZ() {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(MPU_GYRO_Z_H);
    Wire.endTransmission(false);
    // On caste l'adresse en uint16_t pour cibler la bonne fonction ESP32
    Wire.requestFrom((uint16_t)MPU_ADDR, (uint8_t)2, true);

    if (Wire.available() < 2) return 0;
    int16_t hi = Wire.read();
    int16_t lo = Wire.read();
    return (hi << 8) | (uint8_t)lo;
}

void GyroTurnTask::start(Movement &mv) {
    finished = false;
    integ = 0.0f;
    lastErr = 0.0f;
    gzFilt = 0.0f;

    mv.stop();
    delay(50);

    imuBegin();

    // 🔧 calibration gyro
    const int samples = 2000;
    long sum = 0;
    for (int i = 0; i < samples; i++) {
        sum += readRawGyroZ();
        delay(2);
    }
    gyroBiasDegPerSec = (sum / (float)samples) / MPU_SENS_250DPS;

    // 🎯 target
    targetHeading = requestedAngle;
    if (targetHeading > 180.0f) targetHeading -= 360.0f;
    if (targetHeading < -180.0f) targetHeading += 360.0f;
}

void GyroTurnTask::update(Movement &mv) {
    if (finished) return;

    // ⏱ dt
    unsigned long now = micros();
    float dt = (now - lastMicros) / 1e6f;
    if (dt <= 0.0f) dt = 0.001f;
    lastMicros = now;

    // 📐 gyro
    float gz = readRawGyroZ() / MPU_SENS_250DPS;
    gz -= gyroBiasDegPerSec;

    // 🧹 low-pass filter
    const float alpha = 0.9f;
    gzFilt = alpha * gzFilt + (1.0f - alpha) * gz;

    headingDeg += gzFilt * dt;

    if (headingDeg > 180.0f) headingDeg -= 360.0f;
    if (headingDeg < -180.0f) headingDeg += 360.0f;

    // ❌ error
    float err = targetHeading - headingDeg;
    if (err > 180.0f) err -= 360.0f;
    if (err < -180.0f) err += 360.0f;

    // 🛑 stop condition
    if (fabs(err) < 1.0f && fabs(gzFilt) < 0.5f) {
        mv.stop();
        finished = true;
        return;
    }

    // 🎛 PID
    integ += err * dt;
    integ = constrain(integ, -30.0f, 30.0f);

    float deriv = (err - lastErr) / dt;
    lastErr = err;

    float out = Kp * err + Ki * integ + Kd * deriv;
    int pwm = constrain(abs(out), 25, maxPWM);

    // 🔁 motors
    mv.motorLeft->setSpeed(pwm);
    mv.motorRight->setSpeed(pwm);

    if (err > 0) {
        // Cas 1 : Pivot (ex: Tourner à Droite)
        // Le moteur gauche avance, le droit recule
        mv.motorLeft->forward();
        mv.motorRight->backward();
    } else {
        // Cas 2 : Pivot inverse (ex: Tourner à Gauche)
        // Le moteur gauche recule, le droit avance
        mv.motorLeft->backward();
        mv.motorRight->forward();
    }
}

void GyroTurnTask::cancel(Movement &mv) {
    mv.stop();
    finished = true;
}
