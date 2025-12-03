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

    // setup gyro and accelerometer
    
    
    #if DEBUG_MOVEMENT
    // Serial.println("=== Configuration du robot ===");
    // Serial.print("Diametre roues: "); Serial.print(wheelDiameter); Serial.println(" cm");
    // Serial.print("Distance entre roues: "); Serial.print(wheelBase); Serial.println(" cm");
    // Serial.print("Resolution encodeur: "); Serial.print(encoderResolution); Serial.println(" ticks/tour");
    // Serial.print("Circonference roue: "); Serial.print(wheelCircumference); Serial.println(" cm");
    // Serial.println("==============================");
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
//     float Kp = 0.4f; // Ajustez ces valeurs selon vos tests
//     float Ki = 0.4f; // Ajustez ces valeurs selon vos tests
//     float Kd = 0.4f; // Non utilisé dans cette version
//     float target = 0;
//     float error = 0;    
//     float integral = 0;
//     float derivatieve = 0;
//     float lastError = 0;    
//     float angle = 0;    

//     int gyro_X, gyro_Y, gyro_Z;
//     long gyro_x_cal, gyro_y_cal, gyro_z_cal;
//     boolean setgyroangle;

//     long acc_x, acc_y, acc_z, acc_total_vector;
//     float angle_roll_acc, angle_pitch_acc;
//     float angle_pitch, angle_roll;
//     int angle_pitch_buffer, angle_roll_buffer;  
//     float angle_pitch_output, angle_roll_output;

//     long loop_timer;
//     int temp ;
//     int state = 0;

//     Wire.beginTransmission(0x68); 
//     //Send the requested starting register                                       
//     Wire.write(0x6B);  
//     //Set the requested starting register                                                  
//     Wire.write(0x00);
//     //End the transmission                                                    
//     Wire.endTransmission(); 
                                                
//     //Configure the accelerometer (+/-8g)
    
//     //Start communicating with the MPU-6050
//     Wire.beginTransmission(0x68); 
//     //Send the requested starting register                                       
//     Wire.write(0x1C);   
//     //Set the requested starting register                                                 
//     Wire.write(0x10); 
//     //End the transmission                                                   
//     Wire.endTransmission(); 
                                                
//     //Configure the gyro (500dps full scale)
    
//     //Start communicating with the MPU-6050
//     Wire.beginTransmission(0x68);
//     //Send the requested starting register                                        
//     Wire.write(0x1B);
//     //Set the requested starting register                                                    
//     Wire.write(0x08); 
//     //End the transmission                                                  
//     Wire.endTransmission(); 
//     for (int cal_int = 0; cal_int < 1000 ; cal_int ++) {
//         Wire.beginTransmission(0x68);  
//         //Send the requested starting register                                      
//         Wire.write(0x3B);
//         //End the transmission                                                    
//         Wire.endTransmission(); 
//         //Request 14 bytes from the MPU-6050                                  
//         Wire.requestFrom(0x68,14);    
//         //Wait until all the bytes are received                                       
//         while(Wire.available() < 14);
        
//         //Following statements left shift 8 bits, then bitwise OR.  
//         //Turns two 8-bit values into one 16-bit value                                       
//         acc_x = Wire.read()<<8|Wire.read();                                  
//         acc_y = Wire.read()<<8|Wire.read();                                  
//         acc_z = Wire.read()<<8|Wire.read();                                  
//         temp = Wire.read()<<8|Wire.read();                                   
//         gyro_X = Wire.read()<<8|Wire.read();                                 
//         gyro_Y = Wire.read()<<8|Wire.read();                                 
//         gyro_Z = Wire.read()<<8|Wire.read(); 
//         gyro_x_cal += gyro_X;
//         gyro_y_cal += gyro_Y;
//         gyro_z_cal += gyro_Z;
//         delay(3);
//     }
//     gyro_x_cal /= 1000;
//     gyro_y_cal /= 1000;
//     gyro_z_cal /= 1000;

//     loop_timer = micros();

    
    

    

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
//         Wire.beginTransmission(0x68);  
//         //Send the requested starting register                                      
//         Wire.write(0x3B);
//         //End the transmission                                                    
//         Wire.endTransmission(); 
//         //Request 14 bytes from the MPU-6050                                  
//         Wire.requestFrom(0x68,14);    
//         //Wait until all the bytes are received                                       
//         while(Wire.available() < 14);
        
//         //Following statements left shift 8 bits, then bitwise OR.  
//         //Turns two 8-bit values into one 16-bit value                                       
//         acc_x = Wire.read()<<8|Wire.read();                                  
//         acc_y = Wire.read()<<8|Wire.read();                                  
//         acc_z = Wire.read()<<8|Wire.read();                                  
//         temp = Wire.read()<<8|Wire.read();                                   
//         gyro_X = Wire.read()<<8|Wire.read();                                 
//         gyro_Y = Wire.read()<<8|Wire.read();                                 
//         gyro_Z = Wire.read()<<8|Wire.read(); 
//         gyro_X -=   gyro_x_cal;
//         gyro_Y -=   gyro_y_cal;
//         gyro_Z -=   gyro_z_cal; 

//         angle_pitch += gyro_X * 0.0000611;
//         angle_roll  += gyro_Y * 0.0000611;

//         angle_pitch += angle_roll * sin(gyro_Z * 0.000001066);

//         // accelerometter angle calculations

//         acc_total_vector = sqrt((acc_x * acc_x) + (acc_y * acc_y) + (acc_z * acc_z));
//         angle_pitch_acc = asin((float)acc_y / (float)acc_total_vector) * 57.296;
//         angle_roll_acc  = asin((float)acc_x / (float)acc_total_vector) * -57.296;

//         angle_pitch_acc -= 0.0;
//         angle_roll_acc  -= 0.0;

//         if(setgyroangle){
//             angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;
//             angle_roll  = angle_roll  * 0.9996 + angle_roll_acc  * 0.0004;
//         }
//         else{
//             angle_pitch = angle_pitch_acc;
//             angle_roll  = angle_roll_acc;
//             setgyroangle = true;
//         }
//         angle_pitch_output = angle_pitch * 0.90 + angle_pitch_acc * 0.1;
//         angle_roll_output  = angle_roll  * 0.90 + angle_roll_acc  * 0.1;

//         error = target - angle_pitch_output;
//         integral = integral + error;
//         derivatieve = error - lastError;
    
//         angle = Kp * error + Ki * integral + Kd * derivatieve;

//         // Erreur entre les roues
//         Serial.print("angle_pitch_output: ");
//         Serial.println(angle_pitch_output);
//         Serial.print("Error: ");
//         Serial.println(error);
//         Serial.print("Left Ticks: ");
//         Serial.println(leftTicks);
//         Serial.print("Right Ticks: ");
//         Serial.println(rightTicks);

    
        

//         motorLeft->setSpeed(speed);
//         motorRight->setSpeed(speed);
//         motorLeft->run((cm >= 0) ? FORWARD : BACKWARD);
//         motorRight->run((cm >= 0) ? FORWARD : BACKWARD);

//         if(angle_pitch_output > 0){
//             motorLeft->setSpeed(speed - angle);
//             motorRight->setSpeed(speed + angle);
//             motorLeft->run((cm >= 0) ? FORWARD : BACKWARD);
//             motorRight->run((cm >= 0) ? FORWARD : BACKWARD);
//         }
//         else{
//             motorLeft->setSpeed(speed + angle);
//             motorRight->setSpeed(speed - angle);
//             motorLeft->run((cm >= 0) ? FORWARD : BACKWARD);
//             motorRight->run((cm >= 0) ? FORWARD : BACKWARD);

//         }
//         lastError = error ;

//         updateEncoderTimestamps();
        
//     }
    

//     stop();
// }


// void Movement::moveDistance(float cm, int speed) {
//     resetEncoders();
//     CapPID.SetMode(MANUAL);
//     CapOutput = 0;
//     persistentError = 0.0f;
//     long targetTicks = cmToTicks(cm);
//     bool forwardDir = (cm > 0);
    
//     // --- PARAMÈTRES ---
//     float alpha = 0.5f; 
//     float slowDownTicks = 50.0f; 
//     float minSlowFactor = 0.3f;  
    
//     // Sécurité anti-calage : On met 85 ici directement
//     #define MIN_MOTOR_PWM 90

//     int warmupIterations = 50;
//     int loopCounter = 0;
//     // Ajoutez cette variable paramètre au début de la fonction (ou en constante)
//     int rampUpDuration = 80; // Durée de l'accélération en nombre de boucles (approx 0.5 à 1 sec selon votre loop)

//     // *** CONFIGURATION PID (UNE SEULE FOIS AVANT LA BOUCLE) ***
    
//     // 1. Cap PID
//     CapPID.SetMode(AUTOMATIC);
//     CapPID.SetOutputLimits(-100, 100); 
//     CapPID.SetSampleTime(MOVEMENT_LOOP_DELAY); 

//     // 2. Speed PID
//     SpeedPID.SetMode(AUTOMATIC);
//     // Ici on applique la limite de 85 pour empêcher l'effondrement à la fin
//     SpeedPID.SetOutputLimits(MIN_MOTOR_PWM, 255); 
//     SpeedPID.SetSampleTime(MOVEMENT_LOOP_DELAY); 

    
    
//     //----------------------------------------------------------------------
    
//     while (abs(encoderLeft.getTicks()) < abs(targetTicks) && abs(encoderRight.getTicks()) < abs(targetTicks)) {

//         long leftTicks = encoderLeft.getTicks();
//         long rightTicks = encoderRight.getTicks();

//         // --- 1. CONTRÔLE DE VITESSE ---
//         float distanceRemaining = abs(targetTicks) - max(abs(leftTicks), abs(rightTicks));
//         float slowFactor = constrain(distanceRemaining / slowDownTicks, minSlowFactor, 1.0f); 
        
//         SpeedSetpoint = (float)speed * slowFactor * TICKS_PER_PWM_UNIT; 
//         SpeedInput = getAverageSpeedTicks(); 
        
//         //SpeedPID.Compute(); 
//         int basePWM = (int)SpeedOutput; // Le PWM dynamique (min 85)
        
//         // --- 2. CONTRÔLE DE CAP ---
//         float error = leftTicks - rightTicks;
//         persistentError = ( persistentError) +  error;
        
//         CapInput = persistentError; 
//         CapPID.Compute(); 
//         float correction = CapOutput; 

//         // --- 3. APPLICATION ---
//         int leftSpeed = 0;
//         int rightSpeed = 0;

//         if (loopCounter < warmupIterations) {
//              // Warm-up (laisser les moteurs à 0 ou accélération douce si vous en avez une)
//              motorLeft->setSpeed(0);
//              motorRight->setSpeed(0);
//         } else {
//             // --- FEEDFORWARD (FF) ---
//             // Votre réglage final : on booste la gauche de +4
            
//             // --- CALCUL DE LA RAMPE ---
//             int stepsSinceStart = loopCounter - warmupIterations;
//             float rampFactor = (float)stepsSinceStart / (float)rampUpDuration;
//             rampFactor = constrain(rampFactor, 0.0f, 1.0f); 

//             // --- FEEDFORWARD (FF) ---
//             // Vos réglages validés (FF sur droite)
//             int offsetFF = -2; // Ajustez cette valeur selon vos tests
//             int targetBaseRight = basePWM  ; 
//             int targetBaseLeft  = basePWM + offsetFF;

//             // --- CORRECTION "PIÉDESTAL" (CRUCIAL) ---
//             // On ne part pas de 0. On part de MIN_MOTOR_PWM (85).
//             // Formule : Vitesse = Min + (Cible - Min) * Rampe
            
//             int currentBaseLeft = MIN_MOTOR_PWM + (int)((targetBaseLeft - MIN_MOTOR_PWM) * rampFactor);
//             int currentBaseRight = MIN_MOTOR_PWM + (int)((targetBaseRight - MIN_MOTOR_PWM) * rampFactor);

//             // --- ATTÉNUATION DU PID PENDANT LA RAMPE ---
//             // Astuce : Si on est en train d'accélérer, on réduit la violence du PID
//             // pour éviter qu'il ne sur-réagisse aux petits frottements de départ.
//             float rampDamping = 1.0f; // 50% de force PID pendant la rampe
//             float effectiveCorrection = correction * rampDamping;

//             // --- APPLICATION PID ---
//             // Note : On utilise dynamicMinPWM = MIN_MOTOR_PWM car on est déjà au-dessus
//             leftSpeed  = constrain(currentBaseLeft - (int)effectiveCorrection, MIN_MOTOR_PWM, 255);
//             rightSpeed = constrain(currentBaseRight + (int)effectiveCorrection, MIN_MOTOR_PWM, 255);
            
            
            
//             // ... envoi aux moteurs ...
            
//             motorLeft->setSpeed(leftSpeed);
//             motorRight->setSpeed(rightSpeed);
//             motorLeft->run(forwardDir ? FORWARD : BACKWARD);
//             motorRight->run(forwardDir ? FORWARD : BACKWARD);

//             // Logs
//             Serial.print("ErrCap: "); Serial.print(error);
//             // Serial.print(" | encoderL: "); Serial.print(leftTicks);
//             // Serial.print(" | encoderR: "); Serial.print(rightTicks);
//             Serial.print(" | PersErr: "); Serial.print(persistentError);
//             Serial.print(" | CorrCap: "); Serial.print(correction);
//             Serial.print(" | BasePWM: "); Serial.print(basePWM);
//             Serial.print(" | L: "); Serial.print(leftSpeed);
//             Serial.print(" | R: "); Serial.println(rightSpeed);
//             Serial.print(" | Dist: "); Serial.println(getDistanceTraveled());
//         }
//         delay(MOVEMENT_LOOP_DELAY);

//         loopCounter++;
//         updateEncoderTimestamps();
//     }

//     stop();
// }
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









