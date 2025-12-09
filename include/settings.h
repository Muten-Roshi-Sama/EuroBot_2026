#ifndef SETTINGS_H
#define SETTINGS_H

#pragma once

/**
 * SETTINGS.H
 * 
 * Configuration centrale pour le système de mouvement du robot
 * Tous les paramètres physiques et pins sont définis ici
 */

// =========== ACTUATORS PINS ===========
#define LAUNCH_TRIGGER_PIN A0
#define EMERGENCY_PIN A3
#define TEAM_SWITCH_PIN A2



// =========== MOTOR PINS =============
#define MOTOR_LEFT_ID 1      // Motor 1 = roue gauche
#define MOTOR_RIGHT_ID 2     // Motor 2 = roue droite

#define WHEEL_DIAMETER 6.0f // Diamètre des roues en centimètres (60mm)
#define WHEEL_BASE 10.8f // Distance entre les centres des deux roues en centimètres (10cm)
#define ENCODER_RESOLUTION 70 // Résolution de l'encodeur (nombre de ticks par tour complet) - À VÉRIFIER!

// ENCODERS
#define ENCODER_PIN_LEFT 2   // Arduino Uno: pins 2 et 3 supportent les interruptions
#define ENCODER_PIN_RIGHT 3

// SPEED
#define DEFAULT_SPEED 120  // Vitesse par défaut des moteurs (0 = arrêt, 255 = vitesse maximale) min 80
#define MIN_SPEED 80
#define MAX_SPEED 255
#define MOVEMENT_LOOP_DELAY 0.01f
#define STOP_TOLERANCE 2


// =========== STEPPER PINS ===========
#define STEPPER_M1_PIN1 8
#define STEPPER_M1_PIN2 10
#define STEPPER_M1_PIN3 9
#define STEPPER_M1_PIN4 11

#define STEPPER_M2_PIN1 4
#define STEPPER_M2_PIN2 6
#define STEPPER_M2_PIN3 5
#define STEPPER_M2_PIN4 7





// =========== LiDAR ===========
#define LIDAR1_PIN 8   // pin XSHUT du LIDAR1
#define LIDAR2_PIN 9   // pin XSHUT du LIDAR2
#define LIDAR3_PIN 10  // pin XSHUT du LIDAR3

// Seuils de détection d'obstacle (en mm)
#define LIDAR1_THRESHOLD 80
#define LIDAR2_THRESHOLD 80
#define LIDAR3_THRESHOLD 100


#endif // SETTINGS_H