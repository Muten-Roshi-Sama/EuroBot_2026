
#include "Detection.h"

#define MAX_DISTANCE 200  // Maximum distance for ultrasonic in cm

Detection::Detection(uint8_t trigPin, uint8_t echoPin, uint8_t irPin)
    : _trigPin(trigPin), _echoPin(echoPin), _irPin(irPin), distance(0), obstacleDetected(false) {
    sonar = new NewPing(_trigPin, _echoPin, MAX_DISTANCE);
}

void Detection::begin() {
    pinMode(_irPin, INPUT);
    Serial.println("Detection sensors initialized");
}

void Detection::update() {
    // Read ultrasonic distance
    distance = sonar->ping_cm();  // Returns distance in cm
    if (distance == 0) distance = MAX_DISTANCE; // handle no reading

    // Read IR sensor
    int irValue = digitalRead(_irPin);
    obstacleDetected = (irValue == HIGH || distance < 20); // Threshold 20 cm

    // Debug print
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.print(" cm, Obstacle: ");
    Serial.println(obstacleDetected ? "YES" : "NO");
}

float Detection::getDistance() {
    return distance;
}

bool Detection::isObstacleDetected() {
    return obstacleDetected;
}
