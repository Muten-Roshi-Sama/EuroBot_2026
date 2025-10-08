#ifndef DETECTION_H
#define DETECTION_H

#include <Arduino.h>
#include <NewPing.h>  // External library for ultrasonic sensor

class Detection {
public:
    Detection(uint8_t trigPin, uint8_t echoPin, uint8_t irPin);
    void begin();
    void update();
    
    // Sensor readings
    float getDistance();       // Ultrasonic distance in cm
    bool isObstacleDetected(); // True if obstacle detected by IR or ultrasonic

private:
    uint8_t _trigPin;
    uint8_t _echoPin;
    uint8_t _irPin;

    NewPing* sonar;            // Ultrasonic sensor object
    float distance;            // last distance measured
    bool obstacleDetected;     // IR or ultrasonic detection flag
};

#endif
