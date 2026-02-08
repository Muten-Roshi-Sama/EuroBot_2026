#include "Movement.h"
#include <Arduino.h>
#include <PID_v1.h>
#include <Adafruit_MPU6050.h>
#include<Wire.h>
#include <math.h>
#include <L298N.h>
#include <settings.h>
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
Movement::Movement() : 
    // On retire "AFMS(Adafruit_MotorShield())" car il n'existe plus
    encoderLeft(ENCODER_RESOLUTION),
    encoderRight(ENCODER_RESOLUTION) 
{
    // On initialise les pointeurs à NULL (ils seront créés dans begin())
    motorLeft = nullptr;
    motorRight = nullptr;

    // Le reste de vos initialisations de variables reste identique
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
                     int encPinLeft, int encPinRight, int defSpeed,
                     int enA, int in1, int in2,   // Moteur Gauche (ENA, IN1, IN2)
                     int enB, int in3, int in4) { // Moteur Droit (ENB, IN3, IN4)
                        
    // 1. Sauvegarde des paramètres physiques
    wheelDiameter = wheelDiameterCm;
    wheelBase = wheelBaseCm;
    encoderResolution = encResolution;
    encoderPinLeft = encPinLeft;
    encoderPinRight = encPinRight;
    defaultSpeed = defSpeed;
    
    // 2. Calculs géométriques
    wheelCircumference = PI * wheelDiameter;
    
    // 3. Initialisation des Moteurs L298N
    // On nettoie d'abord si ça a déjà été initialisé (pour éviter les fuites de mémoire)
    if (motorLeft != nullptr) delete motorLeft;
    if (motorRight != nullptr) delete motorRight;

    // Création dynamique des objets moteurs avec vos pins
    motorLeft = new L298N(enA, in1, in2);
    motorRight = new L298N(enB, in3, in4);

    // Initialisation de la vitesse PWM (par exemple 0 au début)
    motorLeft->setSpeed(0);
    motorRight->setSpeed(0);

    // Arrêt initial explicite
    motorLeft->stop();
    motorRight->stop();

    Serial.println("Moteurs L298N initialises avec succes");

    // 4. Initialisation des encodeurs (Code inchangé)
    // Note: Assurez-vous que votre classe Encoder a bien cette méthode
    // encoderLeft.changeResolution(encoderResolution); 
    // encoderRight.changeResolution(encoderResolution);
    
    // Configuration des pins des encodeurs
    pinMode(encoderPinLeft, INPUT_PULLUP);
    pinMode(encoderPinRight, INPUT_PULLUP);
    
    // Attachement des interruptions
    // Assurez-vous que minleftEncoderISR et rightEncoderISR sont bien statiques ou globales
    attachInterrupt(digitalPinToInterrupt(encoderPinLeft), minleftEncoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encoderPinRight), rightEncoderISR, CHANGE);
    
    // 5. Initialisation du timestamp
    lastUpdateTime = micros();
    
    #if DEBUG_MOVEMENT
    // Serial.println("=== Configuration L298N ===");
    // Serial.print("Pins Gauche: "); Serial.print(enA); Serial.print(","); Serial.print(in1); Serial.print(","); Serial.println(in2);
    // Serial.print("Pins Droit: "); Serial.print(enB); Serial.print(","); Serial.print(in3); Serial.print(","); Serial.println(in4);
    #endif
}
// ============= MOUVEMENTS BASIQUES (NON-BLOQUANTS) =============







void Movement::stop() {
    motorLeft->stop();
    motorRight->stop();
}

// ============= ROTATIONS SUR PLACE (2 MOTEURS EN SENS OPPOSÉ) =============









// ============= VIRAGES DOUX (UNE ROUE RALENTIT) =============


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

    if (integral > 50) integral = 50;
    if (integral < -50) integral = -50;
    
    float output = Kp * error + Ki * integral ;
    
    // Limiter la sortie pour éviter des vitesses excessives
    if (output > 255) output = 255;
    if (output < 100) output = 100;
    lastUpdateTime = now;
    
    
    

    
    return output;
}


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
            motorLeft->stop();
            motorRight->stop();

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

            // 1. On définit la vitesse (PWM) pour chaque moteur
            motorLeft->setSpeed(leftSpeed);
            motorRight->setSpeed(rightSpeed);

            // 2. On active les moteurs dans la bonne direction
            if (forwardDir) {
                // Si on veut avancer
                motorLeft->forward();
                motorRight->forward();
            } else {
                // Si on veut reculer
                motorLeft->backward();
                motorRight->backward();
            }

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
    moveDistance(cm, defaultSpeed); // Utilise default Kp et Ki
}

volatile int8_t encoderDirection = 1; // 1 = avant, -1 = arrière

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
        
        int speed = constrain(fabs(pidOutput), 50.0f, 140.0f);
          

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
            
            // On définit la vitesse
            motorLeft->setSpeed(speed);
            motorRight->setSpeed(speed);
            
            // Rotation dans un sens (ex: Droite)
            // Gauche avance, Droite recule
            motorLeft->forward();
            motorRight->backward();
        
        } 
        else
         {
        
            encoderDirection = -1;
            
            // On définit la vitesse
            motorLeft->setSpeed(speed);
            motorRight->setSpeed(speed);
            
            // Rotation dans l'autre sens (ex: Gauche)
            // Gauche recule, Droite avance
            motorLeft->backward();
            motorRight->forward();
        
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
float Movement::ticksToRotations(long ticksRight, long ticksLeft) {
    // Calcul du nombre de rotations basé sur la moyenne des ticks des deux roues
    long avgTicks = (ticksRight + ticksLeft) / 2;
    return (float)avgTicks;
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




















//=========================================================   CODE DE SECOUR   =========================================================//
// ============= MOUVEMENTS AVEC DISTANCE (BLOQUANTS) =============

// void Movement::moveDistance(float cm, int speed) {
//     resetEncoders();
//     long targetTicks = cmToTicks(cm);

//     bool forwardDir = (cm > 0);
//     // if (!forwardDir) targetTicks = -targetTicks;

//     // --- PI PARAMETERS ---
//     float Kp = 0.5f; // plus fort que ton 0.4
//     float Ki = 0.6f; // plus agressif
//     //kp =0.4
//     //ki=float Ki = 0.05f;
//     float integral = 0.0;
//     float integralMax = 300.0f; 
//     float deadzone = 2.0f;
//     float dt = MOVEMENT_LOOP_DELAY / 1000.0f;

//     // --- erreur persistante (filtrée) ---
//     float persistentError = 0.0f;   // mémorise l'erreur entre itérations
//     float alpha = 0.3f;                    // 0 = brut, 1 = très filtré

//     int warmupIterations = 50;  
//     int loopCounter = 0;

//     while (abs(encoderLeft.getTicks()) < abs(targetTicks) && abs(encoderRight.getTicks()) < abs(targetTicks)) {

//         long leftTicks  = encoderLeft.getTicks();
//         long rightTicks = encoderRight.getTicks();

//         // --- erreur instantanée ---
//         float error = leftTicks - rightTicks;

//         // --- calcul erreur persistante (filtrée) ---
//         persistentError =  persistentError +  error;

//         // --- intégration + anti-windup ---
//         if (abs(persistentError) <= deadzone) {
//             // dans la deadzone, ne pas intégrer
//         } else {
//             integral += persistentError * dt;
        
//         }
        
        
//         if (integral > integralMax) integral = integralMax;
//         if (integral < -integralMax) integral = -integralMax;

        

//         // --- correction PI (avec erreur persistante) ---
//         float correction = Kp * persistentError + Ki * integral;
//         // --- correction PI (avec erreur persistante) ---
//         // float distanceRemaining = abs(targetTicks) - max(abs(leftTicks), abs(rightTicks));

// // // // // ajuster Kp
// //         float Kp_mod = Kp * constrain(distanceRemaining / 50.0f, 0.1f, 1.0f);

// // // // ajuster Ki
// //         float Ki_mod = Ki * constrain(distanceRemaining / 50.0f, 0.05f, 1.0f);
//         // float factor = distanceRemaining < 80 ? distanceRemaining / 80.0f : 1.0f;
//         // factor = constrain(factor, 0.3f, 1.0f);

//         // float Kp_mod = Kp * factor;
//         // float Ki_mod = Ki * factor;


//         // float correction = Kp_mod * persistentError + Ki_mod * integral;

//         int leftSpeed = 0;
//         int rightSpeed = 0;

//         // --- warm-up ---
//         if (loopCounter < warmupIterations) {
            

//             motorLeft->setSpeed(0);
//             motorRight->setSpeed(0);
//             motorLeft->run(RELEASE);
//             motorRight->run(RELEASE);

//             Serial.print("[Warm-up] Err: "); Serial.print(error);
//             Serial.print(" | PersErr: "); Serial.print(persistentError);
//             Serial.print(" | I: "); Serial.println(integral);
            

//         } else {

//             // --- PI normal après warm-up ---
//             if (persistentError > 0.0f) {
//                 leftSpeed  = constrain(speed - (correction/2), 90, 255);
//                 rightSpeed = constrain(speed + (correction/2), 90, 255);
//             } else {
//                 leftSpeed  = constrain(speed - (correction/2), 90, 255);
//                 rightSpeed = constrain(speed + (correction/2), 90, 255);
//             }
//             //float distanceRemaining = abs(targetTicks) - max(abs(leftTicks), abs(rightTicks));
//             //float slowFactor = constrain(distanceRemaining / 50.0, 0.1, 1.0); // ralentit les 50 derniers ticks
//             //leftSpeed  = constrain(slowFactor*leftSpeed, 70, 255);   // vitesse minimale de 80
//             //rightSpeed = constrain(slowFactor*rightSpeed, 70, 255); // vitesse minimale de 80

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
            
//             // Serial.print(" | Kp_mod: "); Serial.print(Kp_mod);  
//             // Serial.print(" | Ki_mod: "); Serial.println(Ki_mod);
//         }

//         loopCounter++;
//         updateEncoderTimestamps();
//         delay(MOVEMENT_LOOP_DELAY);

        
        
//     }

//     stop();
    

// }






//=========================================================   SUITE CODE DE SECOUR   =========================================================//
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





// void Movement::moveDistance(float cm, int speed) {
//     resetEncoders();
//     long targetTicks = cmToTicks(cm);
    
//     // --- PARAMÈTRES PID ---
//     float Kp = 1.5f;   // Augmenté pour être réactif
//     float Ki = 0.02f;  // Très faible pour éviter l'instabilité
//     float Kd = 0.5f; 
    
//     float error = 0, lastError = 0;
//     float integral = 0;
//     float derivative = 0;
//     float outputCorrection = 0;

//     // --- VARIABLES GYRO ---
//     float angle_z = 0; // On utilise l'axe Z (Yaw) pour la direction
//     long gyro_z_cal = 0;
//     unsigned long previousTime = micros();
    
//     // --- CALIBRATION GYRO ---
//     // (Je garde ta logique mais simplifiée pour Z uniquement)
//     Serial.println("Calibrating Gyro...");
//     for (int i = 0; i < 2000; i++) {
//         Wire.beginTransmission(0x68);
//         Wire.write(0x47); // Registre Gyro Z (High byte)
//         Wire.endTransmission();
//         Wire.requestFrom(0x68, 2);
//         while(Wire.available() < 2);
//         int16_t rawZ = Wire.read() << 8 | Wire.read();
//         gyro_z_cal += rawZ;
//         delay(1);
//     }
//     gyro_z_cal /= 2000;
//     Serial.println("Calibration Done.");

//     // Direction initiale
//     if (cm > 0) {
//         forward(speed);
//     } else {
//         backward(speed);
//         targetTicks = -targetTicks; 
//     }

//     // --- BOUCLE PRINCIPALE ---
//     while (abs(encoderLeft.getTicks()) < targetTicks && abs(encoderRight.getTicks()) < targetTicks) {
        
//         // 1. GESTION DU TEMPS (Delta T)
//         unsigned long currentTime = micros();
//         float dt = (currentTime - previousTime) / 1000000.0; // Convertir en secondes
//         previousTime = currentTime;

//         // 2. LECTURE GYRO Z (Pour la direction gauche/droite)
//         Wire.beginTransmission(0x68);
//         Wire.write(0x47);
//         Wire.endTransmission();
//         Wire.requestFrom(0x68, 2);
        
//         int16_t raw_gyro_z = Wire.read() << 8 | Wire.read();
        
//         // Soustraire l'offset de calibration
//         float gyro_z_dps = (raw_gyro_z - gyro_z_cal) / 65.5; // 65.5 = sensibilité pour 500dps
        
//         // Intégration : Angle = Vitesse * Temps
//         angle_z += gyro_z_dps * dt; 

//         // 3. CALCUL PID
//         // L'objectif est de garder l'angle à 0 (aller tout droit)
//         error = 0 - angle_z; // Target est 0
        
//         integral += error * dt;
//         // Anti-Windup (Empêche l'intégrale d'exploser) 



//         if (integral > 400) integral = 400; 
//         if (integral < -400) integral = -400;

//         derivative = (error - lastError) / dt;
        
//         outputCorrection = (Kp * error) + (Ki * integral) + (Kd * derivative);
//         lastError = error;

//         // 4. APPLICATION MOTEURS (CONSTRAIN)
//         // On limite la vitesse pour rester entre 0 et 255
//         int leftSpeed = constrain(speed - outputCorrection, 0, 255);
//         int rightSpeed = constrain(speed + outputCorrection, 0, 255);

//         motorLeft->setSpeed(leftSpeed);
//         motorRight->setSpeed(rightSpeed);
        
//         // On doit réaffirmer la direction car setSpeed ne le fait pas toujours
//         motorLeft->run((cm >= 0) ? FORWARD : BACKWARD);
//         motorRight->run((cm >= 0) ? FORWARD : BACKWARD);

//         // Debug léger (toutes les 100ms seulement pour ne pas ralentir la boucle)
//         /* static long lastPrint = 0;
//         if (millis() - lastPrint > 100) {
//              Serial.print("AngZ: "); Serial.print(angle_z);
//              Serial.print(" Corr: "); Serial.println(outputCorrection);
//              lastPrint = millis();
//         }
//         */
        
//         updateEncoderTimestamps();
//     }
    
//     stop();git le
// }









