// // lib/detection/Lidar.cpp
// #include "Lidar.h"
// #include <Wire.h>
// #include "settings.h"
// #include "../util/Debug.h"

// LidarSensor::LidarSensor(const LidarConfig* cfgs, uint8_t cnt)
//     : count(cnt) {
//   if (cnt == 0) return;
  
//   // Copy configs
//   for (uint8_t i = 0; i < cnt; i++) {
//     configs.push_back(cfgs[i]);
//     distances.push_back(-2);  // Initialize as out-of-range
//     lidars.push_back(VL53L0X());
//   }
  
//   debugPrintf(DBG_SENSORS, "LidarSensor configured for %u sensors", count);
// }

// LidarSensor::~LidarSensor() {
//   // Cleanup (optional for AVR, but good practice)
// }

// bool LidarSensor::begin() {
//   if (count == 0) return false;
  
//   setupPins();
//   Wire.begin();
//   delay(200);

//   // All sensors off initially
//   for (uint8_t i = 0; i < count; i++) {
//     digitalWrite(configs[i].pin, LOW);
//   }
//   delay(100);

//   // Init each sequentially
//   for (uint8_t i = 0; i < count; i++) {
//     if (!initLidarAt(i)) {
//       debugPrintf(DBG_SENSORS, "LIDAR[%u] init FAILED", i);
//     }
//     delay(50);
//   }

//   // Start continuous mode
//   for (uint8_t i = 0; i < count; i++) {
//     lidars[i].startContinuous(100);
//   }

//   debugPrintf(DBG_SENSORS, "LidarSensor initialized (%u x VL53L0X)", count);
//   return true;
// }

// void LidarSensor::setupPins() {
//   for (uint8_t i = 0; i < count; i++) {
//     pinMode(configs[i].pin, OUTPUT);
//     digitalWrite(configs[i].pin, LOW);
//   }
//   delay(10);
// }

// bool LidarSensor::initLidarAt(uint8_t index) {
//   if (index >= count) return false;
  
//   digitalWrite(configs[index].pin, HIGH);
//   delay(200);
  
//   lidars[index].setTimeout(500);
//   if (!lidars[index].init()) {
//     return false;
//   }
  
//   lidars[index].setAddress(configs[index].i2cAddr);
//   debugPrintf(DBG_SENSORS, "LIDAR[%u] ready (pin=%u addr=0x%02X)", 
//               index, configs[index].pin, configs[index].i2cAddr);
//   return true;
// }

// void LidarSensor::update() {
//   for (uint8_t i = 0; i < count; i++) {
//     int d = lidars[i].readRangeContinuousMillimeters();
//     if (lidars[i].timeoutOccurred()) {
//       distances[i] = -1;  // timeout
//     } else if (d > 8190) {
//       distances[i] = -2;  // out of range
//     } else {
//       distances[i] = d;
//     }
//   }
// }

// uint8_t LidarSensor::getCount() const {
//   return count;
// }

// int LidarSensor::getDistance(uint8_t index) const {
//   if (index >= count) return -1;
//   return distances[index];
// }

// uint16_t LidarSensor::getThreshold(uint8_t index) const {
//   if (index >= count) return 0;
//   return configs[index].threshold;
// }