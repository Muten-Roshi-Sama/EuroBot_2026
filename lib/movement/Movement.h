#ifndef MOVEMENT_H
#define MOVEMENT_H


#include <Arduino.h>
#include "Encoder.h"
#include <Adafruit_MotorShield.h>
#include "settings.h"

class Movement {
public:
    // Moteurs et encodeurs
    Adafruit_MotorShield AFMS;
    Adafruit_DCMotor *motorLeft;
    Adafruit_DCMotor *motorRight;
    
    // Objets encodeurs (utilisation de la classe Encoder existante)
    Encoder encoderLeft;
    Encoder encoderRight;
    
    // Paramètres physiques du robot (stockés localement mais initialisés depuis settings.h)
    float wheelDiameter;        // Diamètre des roues en cm
    float wheelBase;            // Distance entre les roues en cm
    int encoderResolution;      // Nombre de ticks par tour
    float wheelCircumference;   // Circonférence de la roue en cm (calculée)
    
    // Pins des encodeurs
    int encoderPinLeft;
    int encoderPinRight;
    
    // Vitesse par défaut
    int defaultSpeed;
    
    // Timestamps pour calcul de vitesse
    unsigned long lastUpdateTime;
    
    // Fonctions internes
    float ticksToCm(long ticks);
    long cmToTicks(float cm);
    float ticksToDegrees(long ticks);
    long degreesToTicks(float degrees);
    void resetEncoders();
    void updateEncoderTimestamps();
    
public:
    // Constructeur
    Movement();
    
    // Initialisation avec paramètres du robot
    void begin(float wheelDiameterCm, float wheelBaseCm, int encResolution, 
               int encPinLeft, int encPinRight, int defSpeed = 150);
    
    // Mouvements basiques (non-bloquants)
    void forward(int speed);
    void forward();  // Utilise defaultSpeed
    void backward(int speed);
    void backward(); // Utilise defaultSpeed
    void stop();
    
    // Rotation sur place (2 moteurs en sens opposé)
    void rotateLeft(int speed);   // Rotation sur place vers la gauche
    void rotateLeft();
    void rotateRight(int speed);  // Rotation sur place vers la droite
    void rotateRight();
    
    // Mouvements avec une seule roue (pour virages doux)
    void turnLeftSoft(int speed);  // Seul moteur gauche ralentit
    void turnRightSoft(int speed); // Seul moteur droit ralentit
    
    // Mouvements avec distance précise (bloquants)
    void moveDistance(float cm, int speed); // Contrôle PID avec Kp et Ki
    void moveDistance(float cm); // Utilise defaultSpeed
    void rotate(float degrees, int speed); // Rotation de X degrés
    void rotate(float degrees); // Utilise defaultSpeed
    
    // Getters pour les encodeurs
    long getLeftTicks();
    long getRightTicks();
    float getDistanceTraveled(); // Distance moyenne parcourue en cm
    
    // Fonctions de vitesse (utilise les objets Encoder)
    float getLeftRPM();          // Vitesse roue gauche en RPM
    float getRightRPM();         // Vitesse roue droite en RPM
    float getLeftRevolutions();  // Nombre de tours roue gauche
    float getRightRevolutions(); // Nombre de tours roue droite
    
    // Accès direct aux objets encodeurs (pour fonctionnalités avancées)
    Encoder* getLeftEncoder();
    Encoder* getRightEncoder();
    
    // Callbacks statiques pour les interruptions
    static void leftEncoderISR();
    static void rightEncoderISR();
    static void minleftEncoderISR();
    static void minrightEncoderISR();
    float PIDControlAngle(unsigned long& lastUpdateTimeAngle,float targetAngle, float currentAngle, float Kp, float Ki);
    float PIDControlDistance(unsigned long& lastUpdateTimeDist,float targetDistance, float currentDistance, float Kp, float Ki);
    static void encoderLeftISRWrapper();
    static void encoderRightISRWrapper();
    
    
    
    // Instance statique pour accès depuis les ISR
    static Movement* instance;
};

#endif
