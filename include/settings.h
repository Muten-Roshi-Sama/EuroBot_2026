#ifndef SETTINGS_H
#define SETTINGS_H

#pragma once

/**
 * SETTINGS.H (Version ESP32)
 */

// =========== Boutons (Pins Sûres ESP32) ===========
// On évite D12, D0, D2. On prend des GPIO classiques.
#define EMERGENCY_PIN 23 
#define CONTACT_PIN   5  

// ============ Géométrie du Robot =============
#define WHEEL_DIAMETER 6.0f     // 60mm
#define WHEEL_BASE     10.8f    // 10.8cm
#define ENCODER_RESOLUTION 70 // ATTENTION : Mettez la vraie valeur de VOS encodeurs ici (ex: 360, 1440...)

// =========== Encodeurs (Pins Sûres ESP32) ===========
// Arduino Uno utilise 2 et 3. Sur ESP32, c'est dangereux pour le boot.
// On utilise 18 et 19 qui sont parfaits pour ça.
#define ENCODER_PIN_LEFT  18
#define ENCODER_PIN_RIGHT 19

// =========== Vitesse ===========
#define DEFAULT_SPEED 150
#define MIN_SPEED     80
#define MAX_SPEED     255

// =========== Moteurs (L298N sur ESP32) ===========
// Ces IDs ne servent plus avec la lib L298N directe, 
// mais on les garde pour éviter les erreurs si le code les appelle.
#define MOTOR_LEFT_ID  1
#define MOTOR_RIGHT_ID 2

// =========== Calibration ===========
#define DISTANCE_CORRECTION_FACTOR 1.0f
#define ROTATION_CORRECTION_FACTOR 1.0f

// =========== Contrôle ===========
// Correction importante : delay() attend des millisecondes (int), pas des floats.
// 10ms est standard pour une boucle fluide.
#define MOVEMENT_LOOP_DELAY 10 

#define STOP_TOLERANCE 2

// =========== Debug ===========
#define DEBUG_MOVEMENT 1
#define DEBUG_ENCODERS 1

#endif // SETTINGS_H