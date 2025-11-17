#ifndef SETTINGS_H
#define SETTINGS_H

/**
 * SETTINGS.H
 * 
 * Configuration centrale pour le système de mouvement du robot
 * Tous les paramètres physiques et pins sont définis ici
 */



#define WHEEL_DIAMETER 6.0f // Diamètre des roues en centimètres (60mm)
#define WHEEL_BASE 10.8f // Distance entre les centres des deux roues en centimètres (10cm)
#define ENCODER_RESOLUTION 70 // Résolution de l'encodeur (nombre de ticks par tour complet) - À VÉRIFIER!



// CONFIGURATION DES PINS 
#define ENCODER_PIN_LEFT 2   // Arduino Uno: pins 2 et 3 supportent les interruptions
#define ENCODER_PIN_RIGHT 3

// PARAMÈTRES DE VITESSE


#define DEFAULT_SPEED 100  // Vitesse par défaut des moteurs (0 = arrêt, 255 = vitesse maximale) min 80
#define MIN_SPEED 80
#define MAX_SPEED 255

// ============================================
// CONFIGURATION DU MOTOR SHIELD
// ============================================

// Numéros des moteurs sur le Motor Shield
#define MOTOR_LEFT_ID 1      // Motor 1 = roue gauche
#define MOTOR_RIGHT_ID 2     // Motor 2 = roue droite

// ============================================
// FACTEURS DE CORRECTION (CALIBRATION)
// ============================================

// Facteur de correction pour la distance
// 1.0 = pas de correction
// > 1.0 = le robot va trop loin (réduire)
// < 1.0 = le robot ne va pas assez loin (augmenter)
#define DISTANCE_CORRECTION_FACTOR 1.0f

// Facteur de correction pour la rotation
#define ROTATION_CORRECTION_FACTOR 1.0f

// ============================================
// PARAMÈTRES DE CONTRÔLE
// ============================================

// Délai dans les boucles bloquantes (ms)
#define MOVEMENT_LOOP_DELAY 2

// Tolérance pour l'arrêt (ticks)
#define STOP_TOLERANCE 2

// ============================================
// CONFIGURATION DEBUG
// ============================================

// Activer les messages de debug (0 = non, 1 = oui)
#define DEBUG_MOVEMENT 1

// Activer l'affichage des encodeurs
#define DEBUG_ENCODERS 1

#endif // SETTINGS_H
