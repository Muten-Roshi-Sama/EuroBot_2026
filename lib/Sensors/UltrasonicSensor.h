#ifndef ULTRASONIC_SENSOR_H
#define ULTRASONIC_SENSOR_H

#include <Arduino.h>

class UltrasonicSensor {
public:
    UltrasonicSensor(uint8_t trigPin, uint8_t echoPin);

    void begin();
    float getDistanceCm();           // renvoie la distance mesurée
    bool isObstacle(float seuilCm);  // renvoie true si obstacle détecté

private:
    uint8_t trigPin;
    uint8_t echoPin;
};

#endif
