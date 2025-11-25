#ifndef MOVEMENT_H
#define MOVEMENT_H

#include <Arduino.h>
#include "Encoder.h"
#include <Adafruit_MotorShield.h>
#include "settings.h"

// Classe qui gère le mouvement du robot, moteurs et encodeurs
class Movement {
public:
    // Moteurs et Motor Shield
    Adafruit_MotorShield AFMS;       // Objet Motor Shield
    Adafruit_DCMotor *motorLeft;     // Moteur gauche
    Adafruit_DCMotor *motorRight;    // Moteur droit
    
    // Encodeurs pour les roues
    Encoder encoderLeft;
    Encoder encoderRight;
    
    // Paramètres physiques du robot
    float wheelDiameter;        // Diamètre des roues en cm
    float wheelBase;            // Distance entre les roues en cm
    int encoderResolution;      // Nombre de ticks par tour
    float wheelCircumference;   // Circonférence roue = PI*diamètre
    
    // Pins connectés aux encodeurs
    int encoderPinLeft;
    int encoderPinRight;
    
    // Vitesse par défaut pour les fonctions simplifiées
    int defaultSpeed;
    
    // Temps pour calculer l’intervalle entre ticks
    unsigned long lastUpdateTime;
    
    // Fonctions internes pour conversions et gestion encodeurs
    float ticksToCm(long ticks);       // Convertit ticks → cm
    long cmToTicks(float cm);          // Convertit cm → ticks
    float ticksToDegrees(long ticks);  // Convertit ticks → rotation robot en degrés
    long degreesToTicks(float degrees);// Convertit rotation en degrés → ticks
    void resetEncoders();              // Remise à zéro des encodeurs
    void updateEncoderTimestamps();    // Mise à jour timestamp pour calcul vitesse
    
public:
    // Constructeur
    Movement();
    
    // Initialisation avec paramètres du robot
    void begin(float wheelDiameterCm, float wheelBaseCm, int encResolution, 
               int encPinLeft, int encPinRight, int defSpeed = 150);
    
    // Mouvements basiques (non-bloquants)
    void forward(int speed);
    void forward();  
    void backward(int speed);
    void backward(); 
    void stop();
    
    // Rotations sur place (moteurs en sens opposé)
    void rotateLeft(int speed);   
    void rotateLeft();
    void rotateRight(int speed);  
    void rotateRight();
    
    // Virages doux (une roue ralentie)
    void turnLeftSoft(int speed);  
    void turnRightSoft(int speed); 
    
    // Mouvements bloquants avec distance précise
    void moveDistance(float cm, int speed); 
    void moveDistance(float cm); 
    void rotate(float degrees, int speed); 
    void rotate(float degrees); 
    
    // Getters pour encoder
    long getLeftTicks();
    long getRightTicks();
    float getDistanceTraveled(); // Distance moyenne parcourue
    
    // Getters pour vitesse
    float getLeftRPM();
    float getRightRPM();
    float getLeftRevolutions();
    float getRightRevolutions();
    
    // Accès direct aux objets encodeurs
    Encoder* getLeftEncoder();
    Encoder* getRightEncoder();
    
    // Callbacks statiques pour ISR
    static void leftEncoderISR();
    static void rightEncoderISR();
    static void minleftEncoderISR();
    static void minrightEncoderISR();
    static void encoderLeftISRWrapper();
    static void encoderRightISRWrapper();
    
    // PID pour contrôle angle et distance
    float PIDControlAngle(unsigned long& lastUpdateTimeAngle,float targetAngle, float currentAngle, float Kp, float Ki);
    float PIDControlDistance(unsigned long& lastUpdateTimeDist,float targetDistance, float currentDistance, float Kp, float Ki);
    
    // Instance statique pour ISR
    static Movement* instance;
};

#endif
