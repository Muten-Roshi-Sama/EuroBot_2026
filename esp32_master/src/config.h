#ifndef CONFIG_H
#define CONFIG_H

#pragma once
/**
 * CONFIG.H
 * 
 * Configuration centrale pour le système de mouvement du robot
 * Tous les paramètres physiques et pins sont définis ici
 */

// 
#define MATCH_DURATION_MS 50000 // Durée du match en millisecondes (50s pour test, 90s en compèt)


// =================== ACTUATORS PINS ======================
#define LAUNCH_TRIGGER_PIN A0
#define TEAM_SWITCH_PIN A2
// #define EMERGENCY_PIN A3


// ===================== MOTORS ============================
#define IN1 33  // LEFT MOTOR : IN1, IN2, ENA (marron, gris, bleu)
#define IN2 25
#define ENA 32
#define IN3 26  // RIGHT MOTOR : IN3, IN4, ENB (rouge, jaune, purple) 
#define IN4 27
#define ENB 14

#define WHEEL_DIAMETER 6.0f // Diamètre des roues en centimètres (60mm)
#define WHEEL_BASE 10.0f // Distance entre les centres des deux roues en centimètres (10cm)
#define ENCODER_RESOLUTION 70 // Résolution de l'encodeur (nombre de ticks par tour complet) - À VÉRIFIER!

// ==================== ENCODERS ===========================
#define ENC_L_A 14   // Pinout : Red (M+), Black (M-), Encoder_A = Yellow, Encoder_B = White
#define ENC_L_B 13
#define ENC_R_A 12   // Motor 2
#define ENC_R_B 11
// PETIT PAMI
#define ENCODER_PIN_LEFT 34   
#define ENCODER_PIN_RIGHT 35


// ==================== STEPPER ============================
// #define STEPPER_M1_PIN1 8
// #define STEPPER_M1_PIN2 10
// #define STEPPER_M1_PIN3 9
// #define STEPPER_M1_PIN4 11

// #define STEPPER_M2_PIN1 4
// #define STEPPER_M2_PIN2 6
// #define STEPPER_M2_PIN3 5
// #define STEPPER_M2_PIN4 7


// ============ SERVO PINS ===========
// #define SERVO_PIN 1
// #define SERVO_MIN_ANGLE 45
// #define SERVO_MAX_ANGLE 135
// #define SERVO_DELAY_MS 1000



// ===================== ULTRASONIC =====================
#define US_TIMEOUT 20000UL
#define US_TRIG_PIN 12
#define US_ECHO_PIN 13


// ===================== LIDAR =====================
#define LIDAR1_Addr 0x30        // pin XSHUT du LIDAR1
// #define LIDAR1_PIN 8         // pin XSHUT du LIDAR1
// #define LIDAR2_PIN 9         // pin XSHUT du LIDAR2
// #define LIDAR3_PIN 10        // pin XSHUT du LIDAR3
#define LIDAR1_THRESHOLD 80     // detection thresh in cm.
#define LIDAR2_THRESHOLD 80     // detection thresh in cm.
#define LIDAR3_THRESHOLD 100    // detection thresh in cm.




#endif // SETTINGS_H