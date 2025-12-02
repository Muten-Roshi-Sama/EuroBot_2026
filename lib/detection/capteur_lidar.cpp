#include "capteur_lidar.h"
#include <Wire.h>

LidarManager::LidarManager(uint8_t xshutPin, uint8_t address) {
    this->xshutPin = xshutPin;
    this->address = address;
}

bool LidarManager::init() {
    Wire.begin();

    pinMode(xshutPin, OUTPUT);
    digitalWrite(xshutPin, HIGH);  // active le capteur
    delay(50);

    if (!sensor.init()) {
        Serial.println("⚠ Erreur init Lidar !");
        return false;
    }

    sensor.setAddress(address);
    Serial.print("Lidar initialisé avec adresse 0x");
    Serial.println(address, HEX);

    return true;
}

uint16_t LidarManager::readDistance() {
    uint16_t distance = sensor.readRangeSingleMillimeters();

    if (sensor.timeoutOccurred()) {
        Serial.println("⚠ Timeout !");
        return 0;
    }

    return distance;
}
