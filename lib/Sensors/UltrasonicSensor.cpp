#include "UltrasonicSensor.h"

UltrasonicSensor::UltrasonicSensor(uint8_t trigPin, uint8_t echoPin)
    : trigPin(trigPin), echoPin(echoPin) {}

void UltrasonicSensor::begin() {
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
}

float UltrasonicSensor::getDistanceCm() {
    // impulse trigger
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);

    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    // lire echo
    long duration = pulseIn(echoPin, HIGH, 25000); // timeout 25 ms (~4m)

    if (duration == 0) return -1; // pas de retour → trop loin ou erreur

    float distance = duration * 0.0343 / 2.0; // vitesse du son / aller-retour
    return distance;
}

bool UltrasonicSensor::isObstacle(float seuilCm) {
    float d = getDistanceCm();
    return (d > 0 && d <= seuilCm);
}
