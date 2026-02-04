#include "Gyro.h"
#include <Wire.h>
#include <Arduino.h>
#include <i2c_addr.h>

// MPU registers (same as in your gyro.cpp)
#define MPU_ADDR GYRO_ADDR

void Gyro::begin() {
  Wire.begin();
  // Wake up MPU
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  lastMicros = micros();
}

void Gyro::readRaw() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x43); // gyro X high byte register
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true);
  int16_t gx = (Wire.read() << 8) | Wire.read();
  int16_t gy = (Wire.read() << 8) | Wire.read();
  int16_t gz = (Wire.read() << 8) | Wire.read();
  rawGyroZ = gz;
}

// samples in begin or calibrate should be done with robot stationary
void Gyro::calibrate(unsigned long samples) {
  long sum = 0;
  for (unsigned long i = 0; i < samples; ++i) {
    readRaw();
    sum += rawGyroZ;
    delay(2);
  }
  float avgRaw = (float)sum / (float)samples;
  // raw to deg/s: MPU default FS 250 dps -> sensitivity 131 LSB/(deg/s)
  gyroZBias = avgRaw / 131.0f;
  // reset integrator
  headingDeg = 0.0f;
  lastMicros = micros();
}

void Gyro::update() {
  readRaw();
  unsigned long now = micros();
  float dt = (now - lastMicros) / 1e6f;
  lastMicros = now;
  // convert raw to deg/s and subtract bias
  float gz = ((float)rawGyroZ) / 131.0f; // deg/s
  float gzCorrected = gz - gyroZBias;
  headingDeg += gzCorrected * dt; // integrate yaw rate
  // normalize to [-180,180] or [0,360]
  if (headingDeg > 180.0f) headingDeg -= 360.0f;
  if (headingDeg <= -180.0f) headingDeg += 360.0f;
}

float Gyro::getHeading() const {
  return headingDeg;
}

void Gyro::resetHeading(float ref) {
  headingDeg = ref;
}



class SimplePID {
public:
    float Kp=1.0f, Ki=0.0f, Kd=0.0f;
    float integrator=0, lastErr=0, outMin=-255, outMax=255;
    unsigned long lastMs = 0;
    void begin(float kp,float ki,float kd){ Kp=kp;Ki=ki;Kd=kd; lastMs=millis(); }
    float update(float err) {
        unsigned long now = millis();
        float dt = (now - lastMs)/1000.0f;
        lastMs = now;
        if (dt <= 0) dt = 0.001f;
        integrator += err * dt;
        float deriv = (err - lastErr)/dt;
        lastErr = err;
        float out = Kp*err + Ki*integrator + Kd*deriv;
        if (out < outMin) out = outMin;
        if (out > outMax) out = outMax;
        return out;
    }
    void reset() { integrator = lastErr = 0; lastMs = millis(); }
};