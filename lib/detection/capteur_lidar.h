#ifndef CAPTEUR_LIDAR_H
#define CAPTEUR_LIDAR_H

#include <Arduino.h>
#include <VL53L0X.h>

class LidarManager {
public:
    LidarManager(uint8_t xshutPin, uint8_t address);

    bool init();
    uint16_t readDistance();

private:
    VL53L0X sensor;
    uint8_t xshutPin;
    uint8_t address;
};

#endif
