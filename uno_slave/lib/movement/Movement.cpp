#include "Movement.h"
#include <Arduino.h>
#include <Adafruit_MotorShield.h>
#include <PID_v1.h>
#include <Adafruit_MPU6050.h>
#include<Wire.h>
#include <math.h>
// --- PID 1 : Contrôleur de Cap (Boucle Interne) ---
// Objectif: Rouler droit (Erreur de Ticks = 0)
double CapSetpoint = 0; 
double CapInput;        // Erreur persistante (Ticks Gauche - Ticks Droit)
double CapOutput;       // Correction directionnelle (PWM +/-)
// Gains : Kp élevé pour réactivité, Ki faible pour éviter le windup, Kd pour l'amortissement.
PID CapPID(&CapInput, &CapOutput, &CapSetpoint, 0.8, 0.6, 0.8, REVERSE);
#define TICKS_PER_PWM_UNIT 0.58f
// 0.6, 0.8, 0.3

// --- PID 2 : Contrôleur de Vitesse (Boucle Externe) ---
// Objectif: Maintenir la vitesse moyenne (en Ticks/dt)
double SpeedSetpoint;   // Vitesse Consigne (Dynamique)
double SpeedInput;      // Vitesse moyenne actuelle mesurée (Ticks/dt)
double SpeedOutput = 120;     // PWM de base (BasePWM) pour les deux moteurs
// Gains : Kp et Ki sont plus faibles et progressifs. Kd est souvent 0 pour la vitesse.
PID SpeedPID(&SpeedInput, &SpeedOutput, &SpeedSetpoint, 0.5, 0.01, 0.0, DIRECT);

// Variable pour le filtre de cap
float persistentError = 0.0f; 

// --- Fonction à implémenter vous-même ---
// Cette fonction DOIT retourner la vitesse moyenne du robot en Ticks/Loop_Time.
// (Ex: (Ticks_L_delta + Ticks_R_delta) / 2)
extern float (getAverageSpeedTicks);
// Instance statique pour les callbacks d'interruption
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
                        int encPinLeft, int encPinRight, int defSpeed) {
                            
    // Sauvegarde des paramètres
    wheelDiameter = wheelDiameterCm;
    wheelBase = wheelBaseCm;
    encoderResolution = encResolution;
    encoderPinLeft = encPinLeft;
    encoderPinRight = encPinRight;
    defaultSpeed = defSpeed;
    
    wheelCircumference = PI * wheelDiameter; // circonférence
    
    Serial.println("Initializing Motor Shield...");
    unsigned long t0 = millis();
    bool shield_ok = false;
    while (millis() - t0 < 2000) { // 2s max
        if (AFMS.begin()) {  // defaults to 0x60, 1.6kHz PWM
            shield_ok = true;
            break;
        }
        delay(10);
    }
    if (!shield_ok) {
        Serial.println("WARN: Motor Shield non detecte (timeout)");
        // continue without motors to avoid blocking
        motorLeft = nullptr;
        motorRight = nullptr;
        return;
    }
    
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
    attachInterrupt(digitalPinToInterrupt(encoderPinLeft), minleftEncoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encoderPinRight), rightEncoderISR, CHANGE);
    
    // Initialisation du timestamp
    lastUpdateTime = micros();
    
    // Arrêt initial des moteurs
    stop();

    // setup gyro and accelerometer
    
    
    // #if DEBUG_MOVEMENT
    // Serial.println("=== Configuration du robot ===");
    // Serial.print("Diametre roues: "); Serial.print(wheelDiameter); Serial.println(" cm");
    // Serial.print("Distance entre roues: "); Serial.print(wheelBase); Serial.println(" cm");
    // Serial.print("Resolution encodeur: "); Serial.print(encoderResolution); Serial.println(" ticks/tour");
    // Serial.print("Circonference roue: "); Serial.print(wheelCircumference); Serial.println(" cm");
    // Serial.println("==============================");
    // #endif
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
    float dt = (now - lastUpdateTimeAngle) / 1e6f;
    if (dt <= 0.0001f) dt = 0.0001f;  // sécurité

    static float integral = 0.0f;
    static float lastError = 0.0f;

    // --- Erreur d'angle normalisée ---
    float error = fmodf((targetAngle - currentAngle + 540.0f), 360.0f) - 180.0f;

    // --- Intégrale avec anti-windup ---
    if (fabs(error) < 5.0f)
    {
        // dans la deadzone, ne pas intégrer

    }
    else
    {
        integral += error * dt;
    }
    
    integral = constrain(integral, -60.0f, 60.0f);

    // --- Dérivée légère (filtrée) ---
    float derivative = (error - lastError) / dt;
    derivative = 0.7f * derivative + 0.3f * (error - lastError); // filtrage

    // --- Sortie PI(D) ---
    float output = Kp * error + Ki * integral;
    output = constrain(output, -255.0f, 255.0f);

    output = abs(output);

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
    integral = constrain(integral, -50.0f, 50.0f);
    
    float output = Kp * error + Ki * integral ;
    output = constrain(output, 100, 255); // limitation vitesse
    lastUpdateTime = now;
    return output;
}

// ============= MOUVEMENTS AVEC DISTANCE (BLOQUANTS) =============

void Movement::moveDistance(float cm, int speed) {
    resetEncoders();
    long targetTicks = cmToTicks(cm);
    bool forwardDir = (cm > 0);
    // if (!forwardDir) targetTicks = -targetTicks;

    // --- PI PARAMETERS ---
    float Kp = 0.5f; // plus fort que ton 0.4
    float Ki = 0.6f; // plus agressif
    //kp =0.4
    //ki=float Ki = 0.05f;
    float integral = 0.0;
    float integralMax = 300.0f; 
    float deadzone = 2.0f;
    float dt = MOVEMENT_LOOP_DELAY / 1000.0f;

    // --- erreur persistante (filtrée) ---
    float persistentError = 0.0f;   // mémorise l'erreur entre itérations
    float alpha = 0.3f;                    // 0 = brut, 1 = très filtré

    int warmupIterations = 50;  
    int loopCounter = 0;

    while (abs(encoderLeft.getTicks()) < abs(targetTicks) && abs(encoderRight.getTicks()) < abs(targetTicks)) {

        long leftTicks  = encoderLeft.getTicks();
        long rightTicks = encoderRight.getTicks();

        // --- erreur instantanée ---
        float error = leftTicks - rightTicks;

        // --- calcul erreur persistante (filtrée) ---
        persistentError =  persistentError +  error;

        // --- intégration + anti-windup ---
        if (abs(persistentError) <= deadzone) {
            // dans la deadzone, ne pas intégrer
        } else {
            integral += persistentError * dt;
        
        }
        
        
        if (integral > integralMax) integral = integralMax;
        if (integral < -integralMax) integral = -integralMax;

        

        // --- correction PI (avec erreur persistante) ---
        float correction = Kp * persistentError + Ki * integral;
        // --- correction PI (avec erreur persistante) ---
        // float distanceRemaining = abs(targetTicks) - max(abs(leftTicks), abs(rightTicks));

// // // // ajuster Kp
//         float Kp_mod = Kp * constrain(distanceRemaining / 50.0f, 0.1f, 1.0f);

// // // ajuster Ki
//         float Ki_mod = Ki * constrain(distanceRemaining / 50.0f, 0.05f, 1.0f);
        // float factor = distanceRemaining < 80 ? distanceRemaining / 80.0f : 1.0f;
        // factor = constrain(factor, 0.3f, 1.0f);

        // float Kp_mod = Kp * factor;
        // float Ki_mod = Ki * factor;


        // float correction = Kp_mod * persistentError + Ki_mod * integral;

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
                leftSpeed  = constrain(speed - (correction/2), 90, 255);
                rightSpeed = constrain(speed + (correction/2), 90, 255);
            } else {
                leftSpeed  = constrain(speed - (correction/2), 90, 255);
                rightSpeed = constrain(speed + (correction/2), 90, 255);
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



void Movement::moveDistance(float cm) {
    moveDistance(cm, defaultSpeed);
}

volatile int8_t encoderDirection = 1; // 1 = avant, -1 = arrière

// Rotation robot
void Movement::rotate(float degrees, int baseSpeed) {
    resetEncoders();
    unsigned long lastUpdateTimeAngle = micros();

    // --- Paramètres PID ---
    float Kp = 1.3f;
    float Ki = 0.08f; // Gain intégral légèrement augmenté pour réduire erreur statique
    const float deadZone = 3.5f;  // Zone morte en degrés
     // Anti-windup plus large pour plus de flexibilité

    // --- Interruption encodeurs (une seule fois) ---
    
    
    attachInterrupt(digitalPinToInterrupt(encoderPinRight), encoderRightISRWrapper, RISING);
    attachInterrupt(digitalPinToInterrupt(encoderPinLeft),  encoderLeftISRWrapper, RISING);
    Serial.println("=== Début rotation PID ===");

    
    

    while (true) {
        //float currentAngle = ticksToDegrees(ticksToRotations(encoderRight.getTicks()*encoderDirection, encoderLeft.getTicks()*encoderDirection));
        long avgTicks = (abs(encoderRight.getTicks()) + abs(encoderLeft.getTicks())) / 2;
        float currentAngle = ticksToDegrees(avgTicks);
        
        
        // PID amélioré (avec dérivée légère et anti-windup)
        float pidOutput = PIDControlAngle(lastUpdateTimeAngle, degrees, currentAngle, Kp, Ki);
        
        int speed = constrain(fabs(pidOutput), 50.0f, 90.0f);
          

        float error = degrees - currentAngle;
        
        

        // --- Normalisation de l’erreur entre -180 et +180 ---
       
        

        // --- Zone morte ---
        

        Serial.print("Target: "); Serial.print(degrees);
        Serial.print(" | Angle: "); Serial.print(currentAngle);
        Serial.print(" | PID: "); Serial.print(pidOutput);
        Serial.print(" | Error: "); Serial.println(error);
        Serial.print(" | Speed: "); Serial.println(speed);
        
        
        if (fabs(error) <= deadZone ) {
            break;  // 150 ms de stabilité avant arrêt
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
float Movement::ticksToRotations(long ticksRight, long ticksLeft) {
    // Calcul du nombre de rotations basé sur la moyenne des ticks des deux roues
    long avgTicks = (ticksRight + ticksLeft) / 2;
    return (float)avgTicks;
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
static long lastLeftTicks = 0;
static long lastRightTicks = 0;
const float DT_MS = MOVEMENT_LOOP_DELAY; // Temps de boucle en ms

float Movement::getAverageSpeedTicks() {
    long currentLeftTicks = encoderLeft.getTicks();
    long currentRightTicks = encoderRight.getTicks();

    // Calcul du déplacement (delta) depuis la dernière itération
    long deltaLeft = abs(currentLeftTicks - lastLeftTicks);
    long deltaRight = abs(currentRightTicks - lastRightTicks);

    // Mettre à jour l'état
    lastLeftTicks = currentLeftTicks;
    lastRightTicks = currentRightTicks;

    // Vitesse moyenne en Ticks/ms
    float averageSpeed = (float)(deltaLeft + deltaRight) / 2.0f;
    
    // Normalisation par le temps (pour obtenir la vitesse réelle si le dt n'est pas constant, mais ici dt est constant)
    // float speedPerMS = averageSpeed / DT_MS; 
    
    // Si votre PID travaille en Ticks/Loop, retournez simplement la moyenne du delta
    return averageSpeed; 
}






