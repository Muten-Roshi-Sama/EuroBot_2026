#include "Movement.h"
#include <Arduino.h>
#include <Adafruit_MotorShield.h>

// Instance statique pour accès aux ISR
Movement* Movement::instance = nullptr;

// Constructeur
Movement::Movement() 
    : AFMS(Adafruit_MotorShield()), 
      encoderLeft(ENCODER_RESOLUTION),
      encoderRight(ENCODER_RESOLUTION) 
{
    motorLeft = nullptr;      // Moteurs non initialisés
    motorRight = nullptr;
    wheelDiameter = WHEEL_DIAMETER;     // Défini dans settings.h
    wheelBase = WHEEL_BASE;             // Défini dans settings.h
    encoderResolution = ENCODER_RESOLUTION;
    wheelCircumference = 0;
    encoderPinLeft = ENCODER_PIN_LEFT;
    encoderPinRight = ENCODER_PIN_RIGHT;
    defaultSpeed = DEFAULT_SPEED;
    lastUpdateTime = 0;
    instance = this; // Permet aux ISR d’accéder à cet objet
}

// Initialisation complète avec paramètres physiques et pins
void Movement::begin(float wheelDiameterCm, float wheelBaseCm, int encResolution, 
                     int encPinLeft, int encPinRight, int defSpeed) 
{
    wheelDiameter = wheelDiameterCm;
    wheelBase = wheelBaseCm;
    encoderResolution = encResolution;
    encoderPinLeft = encPinLeft;
    encoderPinRight = encPinRight;
    defaultSpeed = defSpeed;
    
    wheelCircumference = PI * wheelDiameter; // circonférence
    
    // Initialisation du Motor Shield
    if (!AFMS.begin()) {
        Serial.println("ERREUR: Motor Shield non detecte!");
        while (1); // Arrêt si non détecté
    }
    AFMS.begin();
    Serial.println("Motor Shield initialise avec succes");
    
    // Récupération des moteurs
    motorLeft = AFMS.getMotor(MOTOR_LEFT_ID);
    motorRight = AFMS.getMotor(MOTOR_RIGHT_ID);
    
    // Initialisation des encodeurs
    encoderLeft.changeResolution(encoderResolution);
    encoderRight.changeResolution(encoderResolution);
    
    // Configuration des pins en entrée pullup
    pinMode(encoderPinLeft, INPUT_PULLUP);
    pinMode(encoderPinRight, INPUT_PULLUP);
    
    // Attachement des interruptions
    attachInterrupt(digitalPinToInterrupt(encoderPinLeft), leftEncoderISR, RISING);
    attachInterrupt(digitalPinToInterrupt(encoderPinRight), rightEncoderISR, RISING);
    
    lastUpdateTime = micros(); // Timestamp initial
    stop(); // Arrêt initial
    
    #if DEBUG_MOVEMENT
    Serial.println("=== Configuration du robot ===");
    Serial.print("Diametre roues: "); Serial.print(wheelDiameter); Serial.println(" cm");
    Serial.print("Distance entre roues: "); Serial.print(wheelBase); Serial.println(" cm");
    Serial.print("Resolution encodeur: "); Serial.print(encoderResolution); Serial.println(" ticks/tour");
    Serial.print("Circonference roue: "); Serial.print(wheelCircumference); Serial.println(" cm");
    Serial.println("==============================");
    #endif
}

// ================== MOUVEMENTS BASIQUES ==================

void Movement::forward(int speed) {
    motorLeft->setSpeed(speed);
    motorRight->setSpeed(speed);
    motorLeft->run(FORWARD);
    motorRight->run(FORWARD);
}

void Movement::forward() {
    forward(defaultSpeed);
}

void Movement::backward(int speed) {
    motorLeft->setSpeed(speed);
    motorRight->setSpeed(speed);
    motorLeft->run(BACKWARD);
    motorRight->run(BACKWARD);
}

void Movement::backward() {
    backward(defaultSpeed);
}

void Movement::stop() {
    motorLeft->run(RELEASE); // Arrêt moteurs
    motorRight->run(RELEASE);
}

// ================= ROTATIONS SUR PLACE ==================

void Movement::rotateLeft(int speed) {
    motorLeft->setSpeed(speed);
    motorRight->setSpeed(speed);
    motorLeft->run(BACKWARD); // roue gauche recule
    motorRight->run(FORWARD); // roue droite avance
}

void Movement::rotateLeft() {
    rotateLeft(defaultSpeed);
}

void Movement::rotateRight(int speed) {
    motorLeft->setSpeed(speed);
    motorRight->setSpeed(speed);
    motorLeft->run(FORWARD);
    motorRight->run(BACKWARD);
}

void Movement::rotateRight() {
    rotateRight(defaultSpeed);
}

// ================= VIRAGES DOUX ==================

void Movement::turnLeftSoft(int speed) {
    motorLeft->setSpeed(speed / 2); // gauche ralentit
    motorRight->setSpeed(speed);
    motorLeft->run(FORWARD);
    motorRight->run(FORWARD);
}

void Movement::turnRightSoft(int speed) {
    motorLeft->setSpeed(speed);     
    motorRight->setSpeed(speed / 2); // droite ralentit
    motorLeft->run(FORWARD);
    motorRight->run(FORWARD);
}

// ================= PID ANGLE ET DISTANCE ==================
float Movement::PIDControlAngle(unsigned long& lastUpdateTimeAngle, 
                                float targetAngle, float currentAngle, float Kp, float Ki) 
{
    unsigned long now = micros();
    float dt = (now - lastUpdateTimeAngle) / 1e6f; // conversion µs → s
    if (dt <= 0) dt = 1e-3f;

    static float integral = 0.0f;

    // Erreur d'angle normalisée -180°/+180°
    float error = fmodf((targetAngle - currentAngle + 540.0f), 360.0f) - 180.0f;

    integral += error * dt;
    integral = constrain(integral, -50.0f, 50.0f); // anti-windup

    float output = Kp * error + Ki * integral; 
    output = constrain(output, -255.0f, 255.0f);

    lastUpdateTimeAngle = now;
    return output;
}

float Movement::PIDControlDistance(unsigned long& lastUpdateTimeDist,float targetDistance, float currentDistance, float Kp, float Ki) {
    unsigned long now = micros();
    unsigned long interval = now - lastUpdateTimeDist;
    
    static float integral = 0;
    
    float error = targetDistance - currentDistance;
    float dt = interval / 1e6; 
    integral += error * dt;
    integral = constrain(integral, -50.0f, 50.0f);
    
    float output = Kp * error + Ki * integral ;
    output = constrain(output, 100, 255); // limitation vitesse
    lastUpdateTime = now;
    return output;
}

// ================= MOUVEMENTS AVEC DISTANCE (BLOQUANTS) ==================
void Movement::moveDistance(float cm, int speed) {
    resetEncoders();
    long targetTicks = cmToTicks(cm);
    bool forwardDir = (cm > 0);
    if (!forwardDir) targetTicks = -targetTicks;

    static float persistentError = 0.0f;
    float Kp = 0.8f;                     
    float alpha = 0.5f;     
    float leftFactor = 1.00;
    float rightFactor = 0.97;            

    while (abs(encoderLeft.getTicks()) < targetTicks && abs(encoderRight.getTicks()) < targetTicks) {
        long leftTicks = encoderLeft.getTicks();
        long rightTicks = encoderRight.getTicks();

        float currentError = leftTicks - rightTicks;
        persistentError = alpha * persistentError + (1 - alpha) * currentError;

        float correction = Kp * persistentError;

        int leftSpeed  = constrain(speed - correction, 100, 255);
        int rightSpeed = constrain(speed + correction, 100, 255);

        motorLeft->setSpeed(leftSpeed);
        motorRight->setSpeed(rightSpeed);
        motorLeft->run(forwardDir ? FORWARD : BACKWARD);
        motorRight->run(forwardDir ? FORWARD : BACKWARD);

        updateEncoderTimestamps();
        delay(MOVEMENT_LOOP_DELAY);
    }

    stop();
}

void Movement::moveDistance(float cm) {
    moveDistance(cm, defaultSpeed);
}

volatile int8_t encoderDirection = 1; // 1 = avant, -1 = arrière

// Rotation robot
void Movement::rotate(float degrees, int baseSpeed) {
    resetEncoders();
    unsigned long lastUpdateTimeAngle = micros();

    float Kp = 0.8f;
    float Ki = 0.01f;

    attachInterrupt(digitalPinToInterrupt(encoderPinLeft), encoderLeftISRWrapper, RISING);
    attachInterrupt(digitalPinToInterrupt(encoderPinRight), encoderRightISRWrapper, RISING);

    while (true) {
        float currentAngle = ticksToDegrees(encoderRight.getTicks());
        float pidOutput = PIDControlAngle(lastUpdateTimeAngle, degrees, currentAngle, Kp, Ki);
        int speed = constrain(abs(pidOutput), 60, 255);

        float error = degrees - currentAngle;
        encoderDirection = (error >= 0) ? 1 : -1;

        if (error > 0) {
            motorLeft->setSpeed(fabs(speed));
            motorRight->setSpeed(fabs(speed));
            motorLeft->run(FORWARD);
            motorRight->run(BACKWARD);
        } else {
            motorLeft->setSpeed(fabs(speed));
            motorRight->setSpeed(fabs(speed));
            motorLeft->run(BACKWARD);
            motorRight->run(FORWARD);
        }

        updateEncoderTimestamps();
        delay(MOVEMENT_LOOP_DELAY);

        if (error < 5.0f) break; // zone morte ±5°
    }

    stop();
}

void Movement::rotate(float degrees) {
    rotate(degrees, defaultSpeed);
}

// ================= FONCTIONS DE CONVERSION ==================
float Movement::ticksToCm(long ticks) {
    return ((float)ticks / encoderResolution) * wheelCircumference;
}

long Movement::cmToTicks(float cm) {
    return (long)((cm / wheelCircumference) * encoderResolution);
}

float Movement::ticksToDegrees(long ticks) {
    float arcLength = ticksToCm(ticks);
    return (arcLength * 360.0) / (PI * wheelBase);
}

long Movement::degreesToTicks(float degrees) {
    float arcLength = (PI * wheelBase * abs(degrees)) / 360.0;
    return cmToTicks(arcLength);
}

// ================= ENCODEURS ==================
void Movement::resetEncoders() {
    encoderLeft.reset();
    encoderRight.reset();
    lastUpdateTime = micros();
}

void Movement::updateEncoderTimestamps() {
    unsigned long now = micros();
    unsigned long interval = now - lastUpdateTime;
    
    encoderLeft.setTimestamp(now);
    encoderLeft.setTickInterval(interval);
    
    encoderRight.setTimestamp(now);
    encoderRight.setTickInterval(interval);
    
    lastUpdateTime = now;
}

// ================= GETTERS ==================
long Movement::getLeftTicks() {
    return encoderLeft.getTicks();
}

long Movement::getRightTicks() {
    return encoderRight.getTicks();
}

float Movement::getDistanceTraveled() {
    long avgTicks = (encoderLeft.getTicks() + encoderRight.getTicks()) / 2;
    return ticksToCm(avgTicks);
}

float Movement::getLeftRPM() {
    updateEncoderTimestamps();
    return encoderLeft.getRPM();
}

float Movement::getRightRPM() {
    updateEncoderTimestamps();
    return encoderRight.getRPM();
}

float Movement::getLeftRevolutions() {
    return encoderLeft.getRevolutions();
}

float Movement::getRightRevolutions() {
    return encoderRight.getRevolutions();
}

Encoder* Movement::getLeftEncoder() {
    return &encoderLeft;
}

Encoder* Movement::getRightEncoder() {
    return &encoderRight;
}

// ================= CALLBACKS ==================
void Movement::leftEncoderISR() {
    if (instance != nullptr) instance->encoderLeft.addTick();
}

void Movement::rightEncoderISR() {
    if (instance != nullptr) instance->encoderRight.addTick();
}

void Movement::minleftEncoderISR() {
    if (instance != nullptr) instance->encoderLeft.subtractTick();
}

void Movement::minrightEncoderISR() {
    if (instance != nullptr) instance->encoderRight.subtractTick();
}

void Movement::encoderLeftISRWrapper() {
    if (instance != nullptr) {
        if (encoderDirection < 0) instance->encoderLeft.addTick();
        else instance->encoderLeft.subtractTick();
    }
}

void Movement::encoderRightISRWrapper() {
    if (instance != nullptr) {
        if (encoderDirection > 0) instance->encoderRight.addTick();
        else instance->encoderRight.subtractTick();
    }
}
