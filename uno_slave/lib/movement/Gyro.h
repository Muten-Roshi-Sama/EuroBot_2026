#pragma once
#include <Arduino.h>

class Gyro {
public:
  Gyro() {}
  void begin();                 // init I2C + MPU registers
  void calibrate(unsigned long samples = 1000); // compute gyro_z bias
  void update();                // call frequently, updates headingDeg
  float getHeading() const;     // degrees, 0..360 (or signed if you prefer)
  void resetHeading(float ref = 0.0f);
private:
  float gyroZBias = 0.0f;       // deg/s
  float headingDeg = 0.0f;
  unsigned long lastMicros = 0;
  // internal helpers that read MPU6050 registers; reuse read_mpu_6050_data
  void readRaw();               // sets rawAccel/gyro members
  int16_t rawGyroZ = 0;
};