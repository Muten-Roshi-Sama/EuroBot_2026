#include "mpu6050.h"
#include <Wire.h>
#include <math.h>
#include "globals.h"


bool MPU::begin(bool accel, bool gyro, bool yaw, bool roll, bool pitch) {
    config.accel = accel;
    config.gyro  = gyro;
    config.yaw   = yaw;
    config.roll  = roll;
    config.pitch = pitch;

    dev.initialize();
    if (!dev.testConnection()) { return false; }

    lastUpdateUs = micros();
    this->yaw = 0.0f;
    return true;
}


void MPU::read(MPUData &out, float dt) {
    int16_t axRaw, ayRaw, azRaw;
    int16_t gxRaw, gyRaw, gzRaw;

    // --- Single I2C read ---
    dev.getMotion6(&axRaw, &ayRaw, &azRaw, &gxRaw, &gyRaw, &gzRaw);

    // --- Convert once ---
    float ax = axRaw / 16384.0f;
    float ay = ayRaw / 16384.0f;
    float az = azRaw / 16384.0f;

    float gx = gxRaw / 131.0f;
    float gy = gyRaw / 131.0f;
    float gz = gzRaw / 131.0f;

    // --- Raw outputs ---
    if (config.accel) { out.ax = ax; out.ay = ay; out.az = az; }
    if (config.gyro) { out.gx = gx; out.gy = gy; out.gz = gz; }

    // ---------- YAW (gyro integration) ----------
    if (config.yaw && config.gyro) {
        yaw += out.gz * dt;
        out.yaw = yaw;
    }

    // ---------- ROLL / PITCH (accel-based) ----------
    if (config.roll && config.accel) { out.roll = atan2(out.ay, out.az) * RAD_TO_DEG; }

    if (config.pitch && config.accel) {
        out.pitch = atan2(-out.ax,
                           sqrt(out.ay*out.ay + out.az*out.az)) * RAD_TO_DEG;
    }

}








// void MPU::readRaw(float &ax, float &ay, float &az,
//                     float &gx, float &gy, float &gz) {
//     int16_t axRaw, ayRaw, azRaw;
//     int16_t gxRaw, gyRaw, gzRaw;

//     dev.getMotion6(&axRaw, &ayRaw, &azRaw, &gxRaw, &gyRaw, &gzRaw);

//     // By default, MPU is conf to +-2g using a 16bits signed int, so raw values are in range -32768 → +32767, and 1g = 16384.
//     // ex. 16384 = 1g, 8192 = 0.5g, -16384 = -1g, etc.
//     ax = axRaw / 16384.0f;      // Range axRaw : -32768 → +32767 for ±2g, so divide by 16384 to get g
//     ay = ayRaw / 16384.0f;
//     az = azRaw / 16384.0f;

//     // Gyro: 1°/s = 131 LSB, so divide raw by 131 to get °/s
//     // Important : gyro gives turning RATE, not angle.
//     gx = gxRaw / 131.0f;
//     gy = gyRaw / 131.0f;
//     gz = gzRaw / 131.0f;
// }



// // Roll : tilt left/right,        (NO.)
// // Pitch : tilt forward/backward, (NO.)
// // Yaw : rotation around vertical axis (THIS is what we need for the HEADING).
// void MPU::readAngles(   float &rollDeg,
//                         float &pitchDeg,
//                         float &yawDeg) {
//     float ax, ay, az;
//     float gx, gy, gz;

//     readRaw(ax, ay, az, gx, gy, gz);

//     // --- Time delta ---
//     unsigned long nowUs = micros();
//     float dt = 0.0f;

//     if (lastUpdateUs != 0) {
//         dt = (nowUs - lastUpdateUs) / 1e6f;
//     }
//     lastUpdateUs = nowUs;

//     // --- Roll & Pitch from accelerometer ---
//     float roll  = atan2(ay, az);
//     float pitch = atan2(-ax, sqrt(ay * ay + az * az));

//     rollDeg  = roll  * 180.0f / PI;
//     pitchDeg = pitch * 180.0f / PI;

//     // --- Yaw from gyro integration ---
//     if (dt > 0.0f) {
//         yaw += gz * dt;   // gz in deg/s
//     }

//     // Optional normalization to [-180, 180]
//     if (yaw > 180.0f) yaw -= 360.0f;
//     if (yaw < -180.0f) yaw += 360.0f;

//     yawDeg = yaw;
// }









// void MPU::readG(float &x, float &y, float &z) {
//     int16_t rx, ry, rz;
//     readRaw(rx, ry, rz);
//     x = rx * scaleFactor;
//     y = ry * scaleFactor;
//     z = rz * scaleFactor;
// }

// void IMU::readAngles(float &rollDeg, float &pitchDeg) {
//     float ax, ay, az;
//     readG(ax, ay, az);

//     float roll  = atan2(ay, az);
//     float pitch = atan2(-ax, sqrt(ay * ay + az * az));

//     rollDeg  = roll  * 180.0f / PI;
//     pitchDeg = pitch * 180.0f / PI;
// }




