#include "Movement.h"
#include <Arduino.h>
#include <Adafruit_MotorShield.h>


// Instance statique pour les callbacks d'interruption
Movement* Movement::instance = nullptr;

// Constructeur
Movement::Movement() : AFMS(Adafruit_MotorShield()), 
                        encoderLeft(ENCODER_RESOLUTION),
                        encoderRight(ENCODER_RESOLUTION) {
    motorLeft = nullptr;
    motorRight = nullptr;
    wheelDiameter = WHEEL_DIAMETER;
    wheelBase = WHEEL_BASE;
    encoderResolution = ENCODER_RESOLUTION;
    wheelCircumference = 0;
    encoderPinLeft = ENCODER_PIN_LEFT;
    encoderPinRight = ENCODER_PIN_RIGHT;
    defaultSpeed = DEFAULT_SPEED;
    lastUpdateTime = 0;
    instance = this;
}

// Initialisation complète avec paramètres du robot
void Movement::begin(float wheelDiameterCm, float wheelBaseCm, int encResolution, 
                        int encPinLeft, int encPinRight, int defSpeed) {
    // Sauvegarde des paramètres
    wheelDiameter = wheelDiameterCm;
    wheelBase = wheelBaseCm;
    encoderResolution = encResolution;
    encoderPinLeft = encPinLeft;
    encoderPinRight = encPinRight;
    defaultSpeed = defSpeed;
    
    // Calcul de la circonférence de la roue
    wheelCircumference = PI * wheelDiameter;
    
    // Initialisation du Motor Shield
    if (!AFMS.begin()) {
        Serial.println("ERREUR: Motor Shield non detecte!");
        while (1); // Arrêt si le shield n'est pas détecté
    }
    AFMS.begin();
    Serial.println("Motor Shield initialise avec succes");
    //AFMS.begin();
    
    // Récupération des moteurs (Motor 1 = gauche, Motor 2 = droite)
    motorLeft = AFMS.getMotor(MOTOR_LEFT_ID);
    motorRight = AFMS.getMotor(MOTOR_RIGHT_ID);
    
    // Initialisation des encodeurs avec la résolution
    encoderLeft.changeResolution(encoderResolution);
    encoderRight.changeResolution(encoderResolution);// verif
    
    // Configuration des pins des encodeurs
    pinMode(encoderPinLeft, INPUT_PULLUP);
    pinMode(encoderPinRight, INPUT_PULLUP);
    
    // Attachement des interruptions
    attachInterrupt(digitalPinToInterrupt(encoderPinLeft), leftEncoderISR, RISING);
    attachInterrupt(digitalPinToInterrupt(encoderPinRight), rightEncoderISR, RISING);
    
    // Initialisation du timestamp
    lastUpdateTime = micros();
    
    // Arrêt initial des moteurs
    stop();
    
    #if DEBUG_MOVEMENT
    Serial.println("=== Configuration du robot ===");
    Serial.print("Diametre roues: "); Serial.print(wheelDiameter); Serial.println(" cm");
    Serial.print("Distance entre roues: "); Serial.print(wheelBase); Serial.println(" cm");
    Serial.print("Resolution encodeur: "); Serial.print(encoderResolution); Serial.println(" ticks/tour");
    Serial.print("Circonference roue: "); Serial.print(wheelCircumference); Serial.println(" cm");
    Serial.println("==============================");
    #endif
}

// ============= MOUVEMENTS BASIQUES (NON-BLOQUANTS) =============

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
    motorLeft->run(RELEASE);
    motorRight->run(RELEASE);
}

// ============= ROTATIONS SUR PLACE (2 MOTEURS EN SENS OPPOSÉ) =============

void Movement::rotateLeft(int speed) {
    // Roue gauche recule, roue droite avance -> rotation sur place vers la gauche
    motorLeft->setSpeed(speed);
    motorRight->setSpeed(speed);
    motorLeft->run(BACKWARD);
    motorRight->run(FORWARD);
}

void Movement::rotateLeft() {
    rotateLeft(defaultSpeed);
}

void Movement::rotateRight(int speed) {
    // Roue gauche avance, roue droite recule -> rotation sur place vers la droite
    motorLeft->setSpeed(speed);
    motorRight->setSpeed(speed);
    motorLeft->run(FORWARD);
    motorRight->run(BACKWARD);
}

void Movement::rotateRight() {
    rotateRight(defaultSpeed);
}

// ============= VIRAGES DOUX (UNE ROUE RALENTIT) =============

void Movement::turnLeftSoft(int speed) {
    // Roue droite à pleine vitesse, roue gauche ralentie
    motorLeft->setSpeed(speed / 2);
    motorRight->setSpeed(speed);
    motorLeft->run(FORWARD);
    motorRight->run(FORWARD);
}

void Movement::turnRightSoft(int speed) {
    // Roue gauche à pleine vitesse, roue droite ralentie
    motorLeft->setSpeed(speed);
    motorRight->setSpeed(speed / 2);
    motorLeft->run(FORWARD);
    motorRight->run(FORWARD);
}
// ============= PID Integration =============
float Movement::PIDControlAngle(unsigned long& lastUpdateTimeAngle,float targetAngle, float currentAngle, float Kp, float Ki) {
    unsigned long now = micros();
    unsigned long interval = now - lastUpdateTimeAngle;

   
    static float integral = 0;
    
    float error = fmodf(((targetAngle - currentAngle) + 540.0f), 360.0f) - 180.0f;
    float dt = interval / 1e6; 
    integral += (error * dt);
    if (integral > 1000) integral = 1000;
    if (integral < -1000) integral = -1000;
    
    
    float output = Kp * error + Ki * integral ;
    
    
    // Limiter la sortie pour éviter des vitesses excessives
    if (output > 360) output = 360;
    if (output < -360) output = -360;
    lastUpdateTime = now;

    
    return output;
    
    
}
float Movement::PIDControlDistance(unsigned long& lastUpdateTimeDist,float targetDistance, float currentDistance, float Kp, float Ki) {
    unsigned long now = micros();
    
    unsigned long interval = now - lastUpdateTimeDist;
    
    static float integral = 0;
    
    float error = targetDistance - currentDistance;
    float dt = interval / 1e6; 
    integral += error * dt;

    if (integral > 100) integral = 100;
    if (integral < -100) integral = -100;
    
    float output = Kp * error + Ki * integral ;
    
    // Limiter la sortie pour éviter des vitesses excessives
    if (output > 255) output = 255;
    if (output < 100) output = 100;
    lastUpdateTime = now;
    
    
    

    
    return output;
}

// ============= MOUVEMENTS AVEC DISTANCE (BLOQUANTS) =============

void Movement::moveDistance(float cm, int speed) {
    resetEncoders();
    long targetTicks = cmToTicks(cm);

    if (cm > 0) {
        forward(speed);
    } else {
        backward(speed);
        targetTicks = -targetTicks; // Valeur absolue pour la comparaison
    }

    // Boucle bloquante jusqu'à atteindre la distance
    while (abs(encoderLeft.getTicks()) < targetTicks && abs(encoderRight.getTicks()) < targetTicks) {
        
        long leftTicks = encoderLeft.getTicks();
        long rightTicks = encoderRight.getTicks();

        // Erreur entre les roues
        long error = leftTicks - rightTicks;
        Serial.print("Error: ");
        Serial.println(error);
        Serial.print("Left Ticks: ");
        Serial.println(leftTicks);
        Serial.print("Right Ticks: ");
        Serial.println(rightTicks);

        // PID simple pour corriger l'écart
        float Kp_wheel = 0.6f; // à ajuster
        int correction = Kp_wheel * error;
        if (abs(error) <= 1) correction = 0;

        int leftSpeed  = constrain(speed - correction, 0, 255);
        int rightSpeed = constrain(speed + correction, 0, 255);
        

        motorLeft->setSpeed(leftSpeed);
        motorRight->setSpeed(rightSpeed);
        motorLeft->run((cm >= 0) ? FORWARD : BACKWARD);
        motorRight->run((cm >= 0) ? FORWARD : BACKWARD);

        updateEncoderTimestamps();
        delay(MOVEMENT_LOOP_DELAY);
    }
    

    stop();

    #if DEBUG_MOVEMENT
    Serial.print("Distance parcourue: ");
    Serial.print(getDistanceTraveled());
    Serial.println(" cm");
    #endif
    // resetEncoders();
    // unsigned long lastUpdateTimeDist = micros();

    // float targetDistance = abs(cm);
    // float Kp = 4.0;  // à ajuster selon ton robot
    // float Ki = 0.5;   // à ajuster aussi (souvent plus petit)

    // while (true) {
    //     // Distance parcourue actuelle
    //     float currentDistance = getDistanceTraveled();

    //     // Erreur = cible - actuelle
    //     float error = targetDistance - currentDistance;

    //     // Condition d’arrêt : proche de la cible
    //     if (fabs(error) <= 1.00f) break;

    //     // PID pour obtenir la consigne de vitesse
    //     float pidOutput = PIDControlDistance(lastUpdateTimeDist, targetDistance, currentDistance, Kp, Ki);

    //     // La direction dépend du signe de l’erreur :
    //     // Si erreur > 0 → aller vers l’avant
    //     // Si erreur < 0 → reculer pour corriger
    //     int direction = (error >= 0) ? FORWARD : BACKWARD;

    //     // Vitesse moteur = amplitude du PID, limitée et plancher minimal
    //     int motorSpeed = constrain(fabs(pidOutput), 0, 255);
       

    //     // Appliquer la consigne aux deux moteurs
    //     motorLeft->setSpeed(motorSpeed);
    //     motorRight->setSpeed(motorSpeed);
    //     motorLeft->run(direction);
    //     motorRight->run(direction);

    //     // Debug (facultatif)
    //     Serial.print("Erreur: "); Serial.print(error);
    //     Serial.print(" | PID: "); Serial.print(pidOutput);
    //     Serial.print(" | Speed: "); Serial.println(motorSpeed);

    //     updateEncoderTimestamps();
    //     delay(MOVEMENT_LOOP_DELAY);
    // }

    // stop();

    // #if DEBUG_MOVEMENT
    // Serial.print("Distance parcourue: ");
    // Serial.print(getDistanceTraveled());
    // Serial.println(" cm");
    // #endif
}



void Movement::moveDistance(float cm) {
    moveDistance(cm, defaultSpeed); // Utilise default Kp et Ki
}

void Movement::rotate(float degrees, int speed) {
    resetEncoders();
    long targetTicks = degreesToTicks(degrees);
    Serial.print(targetTicks);
    Serial.print("\n ");
    
    
    if (degrees > 0) {
        rotateRight(speed); // Rotation dans le sens horaire
    } else {
        rotateLeft(speed); // Rotation dans le sens anti-horaire
        targetTicks = -targetTicks;
    }
    
    // Boucle bloquante jusqu'à atteindre l'angle
    while ( abs(encoderRight.getTicks()) <= targetTicks) {
        // abs(encoderLeft.getTicks()) <= targetTicks || le probleme tant une des 2 conditions est vraie on continue probleme avec decrementation de getLEft + abs cette condition est tjrs fausse ou autre probleme c est condition ne sont jmais vrai au meme 
        //moment
        updateEncoderTimestamps();
        Serial.print(encoderRight.getTicks());
        Serial.print("\n ");
        Serial.print(targetTicks);
        Serial.print("\n ");
        
        
        //Serial.print(encoderRight.getTicks());
        delay(MOVEMENT_LOOP_DELAY);
    }
    
    stop();
    
    #if DEBUG_MOVEMENT
    Serial.print("Rotation effectuee: ");
    Serial.print(degrees);
    Serial.println(" degres");
    #endif
}

void Movement::rotate(float degrees) {
    rotate(degrees, defaultSpeed);
}

// ============= FONCTIONS DE CONVERSION =============

float Movement::ticksToCm(long ticks) {
    // Distance = (ticks / resolution) * circonférence
    return ((float)ticks / encoderResolution) * wheelCircumference;
}

long Movement::cmToTicks(float cm) {
    // Ticks = (distance / circonférence) * resolution
    return (long)((cm / wheelCircumference) * encoderResolution);
}

float Movement::ticksToDegrees(long ticks) {
    // Pour une rotation sur place:
    // Arc parcouru par une roue = (wheelBase * PI * angle) / 360
    // Distance parcourue = ticksToCm(ticks)
    float arcLength = ticksToCm(ticks);
    return (arcLength * 360.0) / (PI * wheelBase);
}

long Movement::degreesToTicks(float degrees) {
    // Arc que doit parcourir une roue pour tourner de X degrés
    float arcLength = (PI * wheelBase * abs(degrees)) / 360.0;
    return cmToTicks(arcLength);
}

void Movement::resetEncoders() {
    encoderLeft.reset();
    encoderRight.reset();
    lastUpdateTime = micros();
}

void Movement::updateEncoderTimestamps() {
    unsigned long now = micros();
    unsigned long interval = now - lastUpdateTime;
    
    // Mise à jour des timestamps pour calcul de vitesse
    encoderLeft.setTimestamp(now);
    encoderLeft.setTickInterval(interval);
    
    encoderRight.setTimestamp(now);
    encoderRight.setTickInterval(interval);
    
    lastUpdateTime = now;
}

// ============= GETTERS =============

long Movement::getLeftTicks() {
    return encoderLeft.getTicks();
}

long Movement::getRightTicks() {
    return encoderRight.getTicks();
}

float Movement::getDistanceTraveled() {
    // Moyenne des deux roues
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

// ============= CALLBACKS D'INTERRUPTION =============

void Movement::leftEncoderISR() {
    if (instance != nullptr) {
        instance->encoderLeft.addTick();
    }
}

void Movement::rightEncoderISR() {
    if (instance != nullptr) {
        instance->encoderRight.addTick();
    }
}
void Movement::minleftEncoderISR()
{
    if (instance != nullptr) {
        instance->encoderLeft.subtractTick();  // ✅
    }
}

void Movement::minrightEncoderISR()
{
    if (instance != nullptr) {
        instance->encoderRight.subtractTick(); // ✅
    }
}


