// // lib/detection/Lidar.h
// #pragma once
// #include <Arduino.h>
// #include <VL53L0X.h>
// #include <vector>

// struct LidarConfig {
//   uint8_t pin;        // XSHUT pin
//   uint8_t i2cAddr;    // I2C address (0x30, 0x31, etc.)
//   uint16_t threshold; // obstacle threshold in mm
// };

// class LidarSensor {
//   public:
//     // Constructor: pass array of configs
//     LidarSensor(const LidarConfig* configs, uint8_t count);
//     ~LidarSensor();
    
//     bool begin();              // Initialize all configured LIDARs
//     void update();             // Poll all sensors, cache distances
    
//     uint8_t getCount() const;  // How many LIDARs configured?
//     int getDistance(uint8_t index) const;  // Get distance by index (mm, or negative for error)
//     uint16_t getThreshold(uint8_t index) const;  // Get threshold for LIDAR
    
//   private:
//     std::vector<VL53L0X> lidars;
//     std::vector<int> distances;
//     std::vector<LidarConfig> configs;
//     uint8_t count;
    
//     void setupPins();
//     bool initLidarAt(uint8_t index);
// };