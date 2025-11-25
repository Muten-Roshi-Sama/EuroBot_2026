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
float Movement::PIDControlAngle(unsigned long& lastUpdateTimeAngle, 
                                float targetAngle, 
                                float currentAngle, 
                                float Kp, 
                                float Ki) 
{
    unsigned long now = micros();
    float dt = (now - lastUpdateTimeAngle) / 1e6f;
    if (dt <= 0.0001f) dt = 0.0001f;  // sécurité

    static float integral = 0.0f;
    static float lastError = 0.0f;

    // --- Erreur d'angle normalisée ---
    float error = fmodf((targetAngle - currentAngle + 540.0f), 360.0f) - 180.0f;

    // --- Intégrale avec anti-windup ---
    integral += error * dt;
    integral = constrain(integral, -60.0f, 60.0f);

    // --- Dérivée légère (filtrée) ---
    float derivative = (error - lastError) / dt;
    derivative = 0.7f * derivative + 0.3f * (error - lastError); // filtrage

    // --- Sortie PI(D) ---
    float output = Kp * error + Ki * integral;
    output = constrain(output, 70.0f, 255.0f);

    lastError = error;
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

    if (integral > 50) integral = 50;
    if (integral < -50) integral = -50;
    
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

    bool forwardDir = (cm > 0);
    if (!forwardDir) targetTicks = -targetTicks;

    // --- PI PARAMETERS ---
    float Kp = 0.5f; // plus fort que ton 0.4
    float Ki = 0.08f; // plus agressif
    //kp =0.4
    //ki=float Ki = 0.05f;
    float integral = 0.0;
    float integralMax = 300.0f; 
    float deadzone = 2.0f;
    float dt = MOVEMENT_LOOP_DELAY / 1000.0f;

    // --- erreur persistante (filtrée) ---
    float persistentError = 0.0f;   // mémorise l'erreur entre itérations
    float alpha = 0.8f;                    // 0 = brut, 1 = très filtré

    int warmupIterations = 50;  
    int loopCounter = 0;

    while (abs(encoderLeft.getTicks()) < abs(targetTicks) && abs(encoderRight.getTicks()) < abs(targetTicks)) {

        long leftTicks  = encoderLeft.getTicks();
        long rightTicks = encoderRight.getTicks();

        // --- erreur instantanée ---
        float error = leftTicks - rightTicks;

        // --- calcul erreur persistante (filtrée) ---
        persistentError = alpha * persistentError + (1.0f - alpha) * error;

        // --- intégration + anti-windup ---
        if (abs(error) <= deadzone) {
            // dans la deadzone, ne pas intégrer
        } else {
            integral += persistentError * dt;
        
        }
        
        
        if (integral > integralMax) integral = integralMax;
        if (integral < -integralMax) integral = -integralMax;

        

        // --- correction PI (avec erreur persistante) ---
        //float correction = Kp * persistentError + Ki * integral;
        // --- correction PI (avec erreur persistante) ---
        float distanceRemaining = abs(targetTicks) - max(abs(leftTicks), abs(rightTicks));

// // // ajuster Kp
//         float Kp_mod = Kp * constrain(distanceRemaining / 50.0f, 0.1f, 1.0f);

// // // ajuster Ki
//         float Ki_mod = Ki * constrain(distanceRemaining / 50.0f, 0.05f, 1.0f);
        float factor = distanceRemaining < 80 ? distanceRemaining / 80.0f : 1.0f;
        factor = constrain(factor, 0.3f, 1.0f);

        float Kp_mod = Kp * factor;
        float Ki_mod = Ki * factor;


        float correction = Kp_mod * persistentError + Ki_mod * integral;

        int leftSpeed = 0;
        int rightSpeed = 0;

        // --- warm-up ---
        if (loopCounter < warmupIterations) {
            

            motorLeft->setSpeed(0);
            motorRight->setSpeed(0);
            motorLeft->run(RELEASE);
            motorRight->run(RELEASE);

            Serial.print("[Warm-up] Err: "); Serial.print(error);
            Serial.print(" | PersErr: "); Serial.print(persistentError);
            Serial.print(" | I: "); Serial.println(integral);

        } else {

            // --- PI normal après warm-up ---
            if (persistentError > 0.0f) {
                leftSpeed  = constrain(speed - (correction), 85, 255);
                rightSpeed = constrain(speed + (correction), 85, 255);
            } else {
                leftSpeed  = constrain(speed - (correction), 85, 255);
                rightSpeed = constrain(speed + (correction), 85, 255);
            }
            //float distanceRemaining = abs(targetTicks) - max(abs(leftTicks), abs(rightTicks));
            //float slowFactor = constrain(distanceRemaining / 50.0, 0.1, 1.0); // ralentit les 50 derniers ticks
            //leftSpeed  = constrain(slowFactor*leftSpeed, 70, 255);   // vitesse minimale de 80
            //rightSpeed = constrain(slowFactor*rightSpeed, 70, 255); // vitesse minimale de 80

            motorLeft->setSpeed(leftSpeed);
            motorRight->setSpeed(rightSpeed);
            motorLeft->run(forwardDir ? FORWARD : BACKWARD);
            motorRight->run(forwardDir ? FORWARD : BACKWARD);

            Serial.print("Err: "); Serial.print(error);
            Serial.print(" | PersErr: "); Serial.print(persistentError);
            Serial.print(" | I: "); Serial.print(integral);
            Serial.print(" | Corr: "); Serial.print(correction);
            Serial.print(" | L: "); Serial.print(leftSpeed);
            Serial.print(" | R: "); Serial.println(rightSpeed);
            Serial.print(" | Distance: "); Serial.println(getDistanceTraveled());
            // Serial.print(" | Kp_mod: "); Serial.print(Kp_mod);  
            // Serial.print(" | Ki_mod: "); Serial.println(Ki_mod);
        }

        loopCounter++;
        updateEncoderTimestamps();

        delay(MOVEMENT_LOOP_DELAY);
        
    }

    stop();
    

}
// void Movement::moveDistance(float cm, int speed) {
//     resetEncoders();
//     long targetTicks = cmToTicks(cm);

//     bool forwardDir = (cm > 0);
//     if (!forwardDir) targetTicks = -targetTicks;

//     // --- PI PARAMETERS ---
//     float Kp = 0.5f;
//     float Ki = 0.02f;
//     float integral = 0.0;
//     float integralMax = 300.0f; 
//     float dt = MOVEMENT_LOOP_DELAY / 1000.0f;
//     int currentSpeed = 90;

//     // --- erreur persistante (filtrée) ---
//     static float persistentError = 0.0f;   // mémorise l'erreur entre itérations
//     float alpha = 0.2f;                    // 0 = brut, 1 = très filtré

//     int warmupIterations = 50;  
//     int loopCounter = 0;

//     while (abs(encoderLeft.getTicks()) < abs(targetTicks) &&
//            abs(encoderRight.getTicks()) < abs(targetTicks)) {
            

//         long leftTicks  = encoderLeft.getTicks();
//         long rightTicks = encoderRight.getTicks();

//         // --- erreur instantanée ---
//         float error = leftTicks - rightTicks;

//         // --- calcul erreur persistante (filtrée) ---
//         persistentError = alpha * persistentError + (1.0f - alpha) * error;

//         // --- intégration + anti-windup ---
//         integral += persistentError * dt;
//         if (integral > integralMax) integral = integralMax;
//         if (integral < -integralMax) integral = -integralMax;

//         // --- correction PI (avec erreur persistante) ---
//         float correction = Kp * persistentError + Ki * integral;

//         int leftSpeed = 0;
//         int rightSpeed = 0;

//         // --- warm-up ---
//         if (loopCounter < warmupIterations) {

//             while(loopCounter < warmupIterations){
//                 motorLeft->setSpeed(0);
//                 motorRight->setSpeed(0);
//                 motorLeft->run(RELEASE);
//                 motorRight->run(RELEASE);

//                 Serial.print("[Warm-up] Err: "); Serial.print(error);
//                 Serial.print(" | PersErr: "); Serial.print(persistentError);
//                 Serial.print(" | I: "); Serial.println(integral);
//                 delay(MOVEMENT_LOOP_DELAY);
//                 loopCounter++;
//             }

            

//         } else {
//             if (currentSpeed < speed)
//             {
//                 currentSpeed += 10;
//                 if (currentSpeed > speed) currentSpeed = speed;
//             }

//             // --- PI normal après warm-up ---
//             if (persistentError > 0.0f) {
                
//                 leftSpeed  = constrain(currentSpeed - correction, 80, 255);
//                 rightSpeed = currentSpeed;
                
//             } else {
//                 leftSpeed  = currentSpeed;
//                 rightSpeed = constrain(currentSpeed + correction, 80, 255);
//             }

//             motorLeft->setSpeed(leftSpeed);
//             motorRight->setSpeed(rightSpeed);
//             motorLeft->run(forwardDir ? FORWARD : BACKWARD);
//             motorRight->run(forwardDir ? FORWARD : BACKWARD);

//             Serial.print("Err: "); Serial.print(error);
//             Serial.print(" | PersErr: "); Serial.print(persistentError);
//             Serial.print(" | I: "); Serial.print(integral);
//             Serial.print(" | Corr: "); Serial.print(correction);
//             Serial.print(" | L: "); Serial.print(leftSpeed);
//             Serial.print(" | R: "); Serial.println(rightSpeed);
//             Serial.print(" | Distance: "); Serial.println(getDistanceTraveled());
//             Serial.print(" | curentSpeed: "); Serial.println(currentSpeed);
//         }

//         loopCounter++;
//         updateEncoderTimestamps();
//         delay(MOVEMENT_LOOP_DELAY);
//     }

//     stop();
// }




// void Movement::moveDistance(float cm, int speed) {
//     resetEncoders();
//     long targetTicks = cmToTicks(cm);

//     if (cm > 0) {
//         forward(speed);
//     } else {
//         backward(speed);
//         targetTicks = -targetTicks; // Valeur absolue pour la comparaison
//     }

//     // Boucle bloquante jusqu'à atteindre la distance
//     while (abs(encoderLeft.getTicks()) < targetTicks && abs(encoderRight.getTicks()) < targetTicks) {
        
//         long leftTicks = encoderLeft.getTicks();
//         long rightTicks = encoderRight.getTicks();

//         // Erreur entre les roues
//         long error = leftTicks - rightTicks;
//         Serial.print("Error: ");
//         Serial.println(error);
//         Serial.print("Left Ticks: ");
//         Serial.println(leftTicks);
//         Serial.print("Right Ticks: ");
//         Serial.println(rightTicks);

//         // PID simple pour corriger l'écart
//         float Kp_wheel = 0.6f; // à ajuster
//         int correction = Kp_wheel * error;
//         if (abs(error) <= 1) correction = 0;

//         int leftSpeed  = constrain(speed - correction, 0, 255);
//         int rightSpeed = constrain(speed + correction, 0, 255);
        

//         motorLeft->setSpeed(leftSpeed);
//         motorRight->setSpeed(rightSpeed);
//         motorLeft->run((cm >= 0) ? FORWARD : BACKWARD);
//         motorRight->run((cm >= 0) ? FORWARD : BACKWARD);

//         updateEncoderTimestamps();
//         delay(MOVEMENT_LOOP_DELAY);
//     }
    

//     stop();

//     #if DEBUG_MOVEMENT
//     Serial.print("Distance parcourue: ");
//     Serial.print(getDistanceTraveled());
//     Serial.println(" cm");
//     #endif
//     // resetEncoders();
//     // unsigned long lastUpdateTimeDist = micros();

//     // float targetDistance = abs(cm);
//     // float Kp = 4.0;  // à ajuster selon ton robot
//     // float Ki = 0.5;   // à ajuster aussi (souvent plus petit)

//     // while (true) {
//     //     // Distance parcourue actuelle
//     //     float currentDistance = getDistanceTraveled();

//     //     // Erreur = cible - actuelle
//     //     float error = targetDistance - currentDistance;

//     //     // Condition d’arrêt : proche de la cible
//     //     if (fabs(error) <= 1.00f) break;

//     //     // PID pour obtenir la consigne de vitesse
//     //     float pidOutput = PIDControlDistance(lastUpdateTimeDist, targetDistance, currentDistance, Kp, Ki);

//     //     // La direction dépend du signe de l’erreur :
//     //     // Si erreur > 0 → aller vers l’avant
//     //     // Si erreur < 0 → reculer pour corriger
//     //     int direction = (error >= 0) ? FORWARD : BACKWARD;

//     //     // Vitesse moteur = amplitude du PID, limitée et plancher minimal
//     //     int motorSpeed = constrain(fabs(pidOutput), 0, 255);
       

//     //     // Appliquer la consigne aux deux moteurs
//     //     motorLeft->setSpeed(motorSpeed);
//     //     motorRight->setSpeed(motorSpeed);
//     //     motorLeft->run(direction);
//     //     motorRight->run(direction);

//     //     // Debug (facultatif)
//     //     Serial.print("Erreur: "); Serial.print(error);
//     //     Serial.print(" | PID: "); Serial.print(pidOutput);
//     //     Serial.print(" | Speed: "); Serial.println(motorSpeed);

//     //     updateEncoderTimestamps();
//     //     delay(MOVEMENT_LOOP_DELAY);
//     // }

//     // stop();

//     // #if DEBUG_MOVEMENT
//     // Serial.print("Distance parcourue: ");
//     // Serial.print(getDistanceTraveled());
//     // Serial.println(" cm");
//     // #endif
// }



void Movement::moveDistance(float cm) {
    moveDistance(cm, defaultSpeed); // Utilise default Kp et Ki
}

volatile int8_t encoderDirection = 1; // 1 = avant, -1 = arrière

void Movement::rotate(float degrees, int baseSpeed) {
    resetEncoders();
    unsigned long lastUpdateTimeAngle = micros();

    // --- Paramètres PID ---
    float Kp = 0.6f;
    float Ki = 0.03f; // Gain intégral légèrement augmenté pour réduire erreur statique
    const float deadZone = 2.0f;  // Zone morte en degrés
     // Anti-windup plus large pour plus de flexibilité

    // --- Interruption encodeurs (une seule fois) ---
    attachInterrupt(digitalPinToInterrupt(encoderPinLeft), encoderLeftISRWrapper, RISING);
    attachInterrupt(digitalPinToInterrupt(encoderPinRight), encoderRightISRWrapper, RISING);

    Serial.println("=== Début rotation PID ===");

    
    unsigned long stableStart = 0;

    while (true) {
        float currentAngle = ticksToDegrees(encoderRight.getTicks());
        float currentAngleLeft = ticksToDegrees(encoderLeft.getTicks());

        // PID amélioré (avec dérivée légère et anti-windup)
        float pidOutput = PIDControlAngle(lastUpdateTimeAngle, degrees, currentAngle, Kp, Ki);
        int speed = constrain(fabs(pidOutput), 0.0f, 255.0f);

        float error = degrees - currentAngle;
        

        // --- Normalisation de l’erreur entre -180 et +180 ---
        error = fmodf((error + 540.0f), 360.0f) - 180.0f;

        // --- Zone morte ---
        if (fabs(error) <= 40.0)
        {
            pidOutput = pidOutput * (fabs(error) / 20.0); // Réduction progressive de la sortie PID
            int speed = constrain(fabs(pidOutput), 50.0f, 255.0f);
        }
        

        // --- Choix de la direction ---
        if (error > 0) {
            encoderDirection = 1;
            motorLeft->setSpeed(speed);
            motorRight->setSpeed(speed);
            motorLeft->run(FORWARD);
            motorRight->run(BACKWARD);
            
            
        } else {
            encoderDirection = -1;
            motorLeft->setSpeed(speed);
            motorRight->setSpeed(speed);
            motorLeft->run(BACKWARD);
            motorRight->run(FORWARD);
            
            
        }
        

        // --- Debug ---
        Serial.print("Target: "); Serial.print(degrees);
        Serial.print(" | Angle: "); Serial.print(currentAngle);
        Serial.print(" | PID: "); Serial.print(pidOutput);
        Serial.print(" | Error: "); Serial.println(error);
        
        if (fabs(error) <= deadZone ) {
             break;  // 150 ms de stabilité avant arrêt
        }
        delay(MOVEMENT_LOOP_DELAY);

        updateEncoderTimestamps();
        
    }

    stop();
    Serial.println("=== Rotation terminée ===");
}
// void Movement::rotate(float degrees, int speed) {
//     resetEncoders();
//     long targetTicks = degreesToTicks(degrees);
//     Serial.print(targetTicks);
//     Serial.print("\n ");


//     if (degrees > 0) {
//         rotateRight(speed); // Rotation dans le sens horaire
//     } else {
//         rotateLeft(speed); // Rotation dans le sens anti-horaire
//         targetTicks = -targetTicks;
//     }

//     // Boucle bloquante jusqu'à atteindre l'angle
//     while ( abs(encoderRight.getTicks()) <= targetTicks -10) {
//         // abs(encoderLeft.getTicks()) <= targetTicks || le probleme tant une des 2 conditions est vraie on continue probleme avec decrementation de getLEft + abs cette condition est tjrs fausse ou autre probleme c est condition ne sont jmais vrai au meme 
//         //moment
//         updateEncoderTimestamps();
//         Serial.print(encoderRight.getTicks());
//         Serial.print("\n ");
//         Serial.print(targetTicks);
//         Serial.print("\n ");


//         //Serial.print(encoderRight.getTicks());
//         delay(MOVEMENT_LOOP_DELAY);
//     }

//     stop();

//     #if DEBUG_MOVEMENT
//     Serial.print("Rotation effectuee: ");
//     Serial.print(degrees);
//     Serial.println(" degres");
//     #endif
// }

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
void Movement::encoderLeftISRWrapper() {
    if (instance != nullptr) {
        if (encoderDirection > 0)
            instance->encoderLeft.addTick();
        else
            instance->encoderLeft.subtractTick();
    }
}

void Movement::encoderRightISRWrapper() {
    if (instance != nullptr) {
        if (encoderDirection > 0)
            instance->encoderRight.addTick();
        else
            instance->encoderRight.subtractTick();
    }
}



