#include "Movement.h"
#include <Arduino.h>
#include "Encoder.h"


#include <Adafruit_MotorShield.h>

Movement::Movement() {}

void Movement::begin( int encoderPin,void minusPulse(), void countPulse()) {
    // Initialize motors, encoders, etc. tout dans le setup
    pinMode(encoderPin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(encoderPin), countPulse, RISING);
    attachInterrupt(digitalPinToInterrupt(encoderPin), minusPulse, RISING);
    
}

void Movement::driveForward(int speed, Adafruit_DCMotor *motor1, Adafruit_DCMotor *motor2, Adafruit_MotorShield AFMS) {
    //Adafruit_MotorShield AFMS = Adafruit_MotorShield();
    //Adafruit_DCMotor *motor1 = AFMS.getMotor(1);
    //Adafruit_DCMotor *motor2 = AFMS.getMotor(2);
    motor1->setSpeed(150);
    motor1->run(FORWARD);
    motor2->setSpeed(150);
    motor2->run(FORWARD);
    
    
}
void Movement::driveBackward(int speed, Adafruit_DCMotor *motor1, Adafruit_DCMotor *motor2, Adafruit_MotorShield AFMS) {
    //Adafruit_MotorShield AFMS = Adafruit_MotorShield();
    //Adafruit_DCMotor *motor1 = AFMS.getMotor(1);
    //Adafruit_DCMotor *motor2 = AFMS.getMotor(2); // mettre dans le setup 
    motor1->setSpeed(150);
    motor1->run(BACKWARD);
    motor2->setSpeed(150);
    motor2->run(BACKWARD);
    
}

void Movement::stop(Adafruit_DCMotor *motor1, Adafruit_DCMotor *motor2, Adafruit_MotorShield AFMS) {
    //Adafruit_MotorShield AFMS = Adafruit_MotorShield();
    //Adafruit_DCMotor *motor1 = AFMS.getMotor(1);
    //Adafruit_DCMotor *motor2 = AFMS.getMotor(2); // mettre dans le setup 
    motor1->run(RELEASE);
    motor2->run(RELEASE);
    // Stop motors
}
void Movement::turnLeft(int speed, Adafruit_DCMotor *motor1, Adafruit_MotorShield AFMS) {
    //Adafruit_MotorShield AFMS = Adafruit_MotorShield();
    //Adafruit_DCMotor *motor1 = AFMS.getMotor(1); // mettre dans le setup 
    motor1->setSpeed(150);
    motor1->run(BACKWARD);
    // Turn left
}
void Movement::turnRight( int speed, Adafruit_DCMotor *motor2, Adafruit_MotorShield AFMS) {
    //Adafruit_MotorShield AFMS = Adafruit_MotorShield();
    //Adafruit_DCMotor *motor2 = AFMS.getMotor(2); // mettre dans le setup 
    motor2->setSpeed(150);
    motor2->run(BACKWARD);
    // Turn right
}
void Movement::getspeed(Encoder encoder,int prevTime) {
    unsigned long now = micros();
    encoder.setTickInterval(now - prevTime);
    prevTime = now;  // mettre dans le setup = micro();
    encoder.setTimestamp(now);
    Serial.println(encoder.getRPM());


}
void Movement::getdist(Encoder encoder)
{
    int dist = encoder.getRPM() * 3.1;
    Serial.println(dist);


}
void Movement::countPulse(Encoder encoder) { 
    //Encoder encoder(100); // mettre dans le setup 
    encoder.addTick();
    
}

void Movement::minusPulse(Encoder encoder) { 
    //Encoder encoder(100); // mettre dans le setup 
    encoder.addTick();
   
    
    
}


