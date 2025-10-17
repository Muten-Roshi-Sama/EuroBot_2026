#ifndef MOVEMENT_H
#define MOVEMENT_H

#include <Arduino.h>
#include "Encoder.h"
#include <Adafruit_MotorShield.h>

class Movement {
public:
    Movement();
    void begin( int encoderPIn,void minusPulse(), void countPulse());
    void driveForward(int speed, Adafruit_DCMotor *motor1, Adafruit_DCMotor *motor2, Adafruit_MotorShield AFMS);
    void stop(Adafruit_DCMotor *motor1, Adafruit_DCMotor *motor2, Adafruit_MotorShield AFMS);
    void getspeed(Encoder encoder,int prevTime);
    void driveBackward(int speed, Adafruit_DCMotor *motor1, Adafruit_DCMotor *motor2, Adafruit_MotorShield AFMS);
    void turnLeft(int speed, Adafruit_DCMotor *motor1, Adafruit_MotorShield AFMS);
    void turnRight( int speed, Adafruit_DCMotor *motor2, Adafruit_MotorShield AFMS);
    void getdist(Encoder encoder);
    void countPulse(Encoder encoder);
    void minusPulse(Encoder encoder);
    
};

#endif
