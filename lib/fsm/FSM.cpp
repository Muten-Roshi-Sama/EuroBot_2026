#include <Arduino.h>
#include "FSM.h"
#include "Movement.h"
#include "settings.h"
#include "capteur_lidar.h"
#include <Adafruit_MotorShield.h>

// ===============================
// INSTANCE MOTEURS
// ===============================
Movement movement;

static void markStateStart(FsmContext &ctx) {
    ctx.stateStartMs = millis();
}

void fsmChangeAction(FsmContext &ctx, FsmAction next) {
    ctx.currentAction = next;
    markStateStart(ctx);
}

void fsmInit(FsmContext &ctx) {
    ctx.currentAction = FsmAction::INIT;
    markStateStart(ctx);
}

void fsmEmergencyStop(FsmContext &ctx) {
    fsmChangeAction(ctx, FsmAction::EMERGENCY_STOP);
}

void fsmDuringAction(FsmContext &ctx, FsmAction action) {
    // Lit la distance du capteur LIDAR
    int d = lireDistance();
    
    // Si obstacle détecté à moins de 5 cm
    if (d >= 0 && d < 5) {
        movement.stop();  // Arrête immédiatement
        fsmChangeAction(ctx, FsmAction::AVOID_OBSTACLE);  // Passe en mode évitement
    }
}

// ===============================
// FSM PRINCIPALE
// ===============================
void fsmStep(FsmContext &ctx) {

    switch (ctx.currentAction) {

    // ============================
    // INIT
    // ============================
    case FsmAction::INIT: {

        Serial.println("=== INIT ===");

        movement.begin(
            WHEEL_DIAMETER,
            WHEEL_BASE,
            ENCODER_RESOLUTION,
            ENCODER_PIN_LEFT,
            ENCODER_PIN_RIGHT,
            DEFAULT_SPEED
        );

        // --- AJOUT : Init LIDAR ---
        Serial.println("Initialisation du LIDAR...");
        if (!initLidar()) {
            Serial.println("ERREUR : Impossible d'initialiser le LIDAR");
            fsmChangeAction(ctx, FsmAction::EMERGENCY_STOP);
            break;
        }
        Serial.println("LIDAR OK");

        fsmChangeAction(ctx, FsmAction::CALIBRATE_ENCODERS);
        break;
    }

    // ============================
    // CALIBRATION
    // ============================
    case FsmAction::CALIBRATE_ENCODERS: {
        calibrateEncoders();
        break;
    }

    // ============================
    // EMERGENCY STOP
    // ============================
    case FsmAction::EMERGENCY_STOP: {
        movement.stop();
        Serial.println("=== EMERGENCY STOP ===");
        break;
    }

    // ============================
    // MOVE FORWARD
    // ============================
    case FsmAction::MOVE_FORWARD: {
        Serial.println("Action : MOVE_FORWARD");

        // Démarre le mouvement de 100 cm
        movement.moveDistance(100);
        
        // Boucle de vérification continue pendant le déplacement
        while (movement.getDistanceTraveled() < 100) {
            // Vérifie s'il y a un obstacle pendant le mouvement
            fsmDuringAction(ctx, FsmAction::CHECK_OBSTACLE);
            
            // Si un obstacle a été détecté, arrête le mouvement
            if (ctx.currentAction == FsmAction::AVOID_OBSTACLE) {
                break;
            }
        }

        // Mouvement terminé sans obstacle → vérification finale
        fsmChangeAction(ctx, FsmAction::CHECK_OBSTACLE);
        break;
    }

    // ============================
    // MOVE BACKWARD
    // ============================
    case FsmAction::MOVE_BACKWARD: {
        movement.backward();
        break;
    }

    // ============================
    // ROTATION 180°
    // ============================
    case FsmAction::TURN_AROUND: {

        Serial.println("Action : TURN_AROUND");

        movement.rotate(-180);

        fsmChangeAction(ctx, FsmAction::MOVE_FORWARD);
        break;
    }

    // ============================
    // CHECK OBSTACLE (LIDAR)
    // ============================
    case FsmAction::CHECK_OBSTACLE: {
    // Lit la distance du capteur LIDAR
        int d = lireDistance();

        Serial.print("Distance LIDAR : ");
        Serial.println(d);

        // Cas erreur de lecture du capteur
        if (d == -1 || d == -2) {
            Serial.println("LIDAR : mesure invalide, on continue");
            break;
        }

        // Détection d'obstacle critique (< 5 cm)
        if (d < 5) {
            Serial.println("OBSTACLE < 5cm → ARRÊT !");
            movement.stop();
            fsmChangeAction(ctx, FsmAction::AVOID_OBSTACLE);
        }

        break;
    }
    // ============================
    // AVOID OBSTACLE
    // ============================
    case FsmAction::AVOID_OBSTACLE: {

        Serial.println("=== AVOID_OBSTACLE ===");
        movement.stop();

        // Version simple : on ne bouge pas encore.
        // Ensuite tu feras variante :
        // - reculer un peu
        // - tourner
        // - réessayer

        break;
    }

    // ============================
    // END
    // ============================
    case FsmAction::END: {
        Serial.println("=== FIN ===");
        movement.stop();
        break;
    }

    // ============================
    // DEFAULT
    // ============================
    default: {
        fsmChangeAction(ctx, FsmAction::INIT);
        break;
    }
    }
}


// ===============================
// CALIBRATION COMPLEXE (inchangée)
// ===============================
void calibrateEncoders() {

    Serial.println("Configuration actuelle:");
    Serial.print("  - Diametre roues: "); Serial.print(WHEEL_DIAMETER); Serial.println(" cm");
    Serial.print("  - Distance entre roues: "); Serial.print(WHEEL_BASE); Serial.println(" cm");
    Serial.print("  - Resolution encodeur: "); Serial.print(ENCODER_RESOLUTION); Serial.println(" ticks/tour");


    Serial.println("\n\n=== TEST 3: VERIFICATION ROTATION ===");
    Serial.println("Le robot va tourner de 180 degres");
    Serial.println("Demarrage dans 5 secondes...\n");

    Serial.println(">>> ROTATION 180 DEGRES <<<");

    movement.moveDistance(20);

    Serial.println("\n=== RESULTATS ROTATION ===");
    Serial.println("Corrections possibles sur WHEEL_BASE...");

    Serial.println("\n\n=========================================");
}



// #include <Arduino.h>
// #include "FSM.h"
// #include "Movement.h"
// #include "settings.h"
// #include "capteur_lidar.h"    // 🔥 AJOUT
// #include <Adafruit_MotorShield.h>

// // Instance du système de mouvement
// Movement movement;

// static void markStateStart(FsmContext &ctx) {
//     ctx.stateStartMs = millis();
// }

// void fsmChangeAction(FsmContext &ctx, FsmAction next) {
//     ctx.currentAction = next;
//     markStateStart(ctx);
// }

// void fsmInit(FsmContext &ctx) {
//     ctx.currentAction = FsmAction::INIT;
//     markStateStart(ctx);
// }

// void fsmEmergencyStop(FsmContext &ctx) {
//     fsmChangeAction(ctx, FsmAction::EMERGENCY_STOP);
// }


// // =============================
// // FSM LOGIC
// // =============================
// void fsmStep(FsmContext &ctx) {
//     switch (ctx.currentAction) {

//     // ---------------------------
//     // INIT
//     // ---------------------------
//     case FsmAction::INIT: {
//         movement.begin(WHEEL_DIAMETER, WHEEL_BASE, ENCODER_RESOLUTION,
//                        ENCODER_PIN_LEFT, ENCODER_PIN_RIGHT, DEFAULT_SPEED);

//         // 🔥 AJOUT : initialisation du LiDAR
//         if (!initLidar()) {
//             Serial.println("ERREUR : Impossible d'initialiser le LIDAR !");
//             fsmChangeAction(ctx, FsmAction::EMERGENCY_STOP);
//             break;
//         }
//         Serial.println("LIDAR OK");

//         fsmChangeAction(ctx, FsmAction::CALIBRATE_ENCODERS);
//         break;
//     }


//     // ---------------------------
//     // CALIBRATION
//     // ---------------------------
//     case FsmAction::CALIBRATE_ENCODERS: {
//         calibrateEncoders();
//         break;
//     }


//     // ---------------------------
//     // EMERGENCY STOP
//     // ---------------------------
//     case FsmAction::EMERGENCY_STOP: {
//         movement.stop();
//         break;
//     }


//     // ---------------------------
//     // MOVE_FORWARD
//     // ---------------------------
//     case FsmAction::MOVE_FORWARD: {
//         Serial.println("Action: MOVE_FORWARD");
//         movement.moveDistance(100);

//         // 🔥 AJOUT : après le mouvement → vérifier obstacle
//         fsmChangeAction(ctx, FsmAction::CHECK_OBSTACLE);
//         break;
//     }


//     // ---------------------------
//     // MOVE_BACKWARD
//     // ---------------------------
//     case FsmAction::MOVE_BACKWARD: {
//         movement.backward();
//         break;
//     }


//     // ---------------------------
//     // TURN_AROUND
//     // ---------------------------
//     case FsmAction::TURN_AROUND: {
//         Serial.println("Action: TURN_AROUND");
//         movement.rotate(-180);
//         fsmChangeAction(ctx, FsmAction::MOVE_FORWARD);
//         break;
//     }


//     // ---------------------------------------------------
//     // 🔥 NOUVEL ÉTAT : CHECK_OBSTACLE (lecture LIDAR)
//     // ---------------------------------------------------
//     case FsmAction::CHECK_OBSTACLE: {
//         int d = lireDistance();

//         if (d == -1 || d == -2) {
//             Serial.println("LIDAR : Pas de mesure");
//             fsmChangeAction(ctx, FsmAction::MOVE_FORWARD);
//             break;
//         }

//         Serial.print("Distance detectee : ");
//         Serial.println(d);

//         // 🔥 SEUIL D’OBSTACLE : 200 mm
//         if (d < 200) {
//             Serial.println("⚠️ OBSTACLE DETECTE !");
//             movement.stop();
//             fsmChangeAction(ctx, FsmAction::AVOID_OBSTACLE);
//         } else {
//             // voie libre → continuer normalement
//             fsmChangeAction(ctx, FsmAction::MOVE_FORWARD);
//         }
//         break;
//     }


//     // ---------------------------------------------------
//     // 🔥 NOUVEL ÉTAT : AVOID_OBSTACLE
//     // ---------------------------------------------------
//     case FsmAction::AVOID_OBSTACLE: {
//         Serial.println("Action : AVOID_OBSTACLE (stop)");

//         movement.stop();

//         // Ici, dans une V2 :
//         // - petit recul
//         // - rotation
//         // - recherche de passage
//         // Pour l’instant, on reste simple.

//         break;
//     }


//     // ---------------------------
//     // END
//     // ---------------------------
//     case FsmAction::END: {
//         movement.stop();
//         break;
//     }


//     // ---------------------------
//     // UNKNOWN STATE
//     // ---------------------------
//     default: {
//         fsmChangeAction(ctx, FsmAction::INIT);
//         break;
//     }
//     }
// }





// void calibrateEncoders() {
//     Serial.println("Configuration actuelle:");
//     Serial.print("  - Diametre roues: "); Serial.print(WHEEL_DIAMETER); Serial.println(" cm");
//     Serial.print("  - Distance entre roues: "); Serial.print(WHEEL_BASE); Serial.println(" cm");
//     Serial.print("  - Resolution encodeur: "); Serial.print(ENCODER_RESOLUTION); Serial.println(" ticks/tour");
    
//     // Serial.println("\n=== TEST 1: MESURE RESOLUTION ENCODEUR ===");
//     // Serial.println("Instructions:");
//     // Serial.println("1. Le robot va avancer pendant 5 secondes");
//     // Serial.println("2. Comptez MANUELLEMENT combien de fois une roue fait un tour COMPLET");
//     // Serial.println("3. Notez le nombre de ticks affiche");
//     // Serial.println("\nDemarrage dans 5 secondes...\n");
//     // delay(5000);
    
//     // // Test de 5 secondes
//     // Serial.println(">>> DEBUT DU TEST <<<");
//     // movement.forward(150);
    
//     // for (int i = 0; i < 50; i++) {
//     //     Serial.print("Ticks gauche: ");
//     //     Serial.print(movement.getLeftTicks());
//     //     Serial.print(" | Ticks droite: ");
//     //     Serial.println(movement.getRightTicks());
//     //     delay(100);
//     // }
    
//     // movement.stop();
    
//     // long ticksLeft = movement.getLeftTicks();
//     // long ticksRight = movement.getRightTicks();
//     // long avgTicks = (ticksLeft + ticksRight) / 2;
    
//     // Serial.println("\n>>> FIN DU TEST <<<");
//     // Serial.println("\n=== RESULTATS ===");
//     // Serial.print("Total ticks gauche: "); Serial.println(ticksLeft);
//     // Serial.print("Total ticks droite: "); Serial.println(ticksRight);
//     // Serial.print("Moyenne: "); Serial.println(avgTicks);
    
//     // Serial.println("\n=== CALCUL DE LA RESOLUTION ===");
//     // Serial.println("Combien de tours COMPLETS avez-vous compte?");
//     // Serial.println("Formule: Resolution = Ticks / Nombre_de_tours");
//     // Serial.println("");
//     // Serial.println("Exemples:");
//     // Serial.print("  Si 1 tour   -> Resolution = "); Serial.print(avgTicks); Serial.println(" ticks/tour");
//     // Serial.print("  Si 2 tours  -> Resolution = "); Serial.print(avgTicks / 2); Serial.println(" ticks/tour");
//     // Serial.print("  Si 3 tours  -> Resolution = "); Serial.print(avgTicks / 3); Serial.println(" ticks/tour");
//     // Serial.print("  Si 4 tours  -> Resolution = "); Serial.print(avgTicks / 4); Serial.println(" ticks/tour");
//     // Serial.print("  Si 5 tours  -> Resolution = "); Serial.print(avgTicks / 5); Serial.println(" ticks/tour");
    
//     // Serial.println("\n\n=== TEST 2: VERIFICATION DISTANCE ===");
//     // Serial.println("Le robot va maintenant essayer d'avancer de 100 cm");
//     // Serial.println("Mesurez la distance REELLE parcourue avec un metre");
//     // Serial.println("\nDemarrage dans 5 secondes...\n");
//     // delay(5000);
    
//     // Serial.println(">>> AVANCE DE 100 CM <<<");
//     // movement.moveDistance(100);
    
//     // Serial.println("\n=== RESULTATS DISTANCE ===");
//     // Serial.print("Distance demandee: 100 cm");
//     // Serial.print("\nQuelle distance REELLE avez-vous mesuree? ___ cm");
//     // Serial.println("\n");
//     // Serial.println("Formule de correction:");
//     // Serial.println("  Nouveau_Diametre = WHEEL_DIAMETER × (Distance_reelle / 100)");
//     // Serial.println("");
//     // Serial.println("Exemples:");
//     // Serial.print("  Si 90 cm  -> Nouveau diametre = "); Serial.print(WHEEL_DIAMETER * 0.90, 2); Serial.println(" cm");
//     // Serial.print("  Si 95 cm  -> Nouveau diametre = "); Serial.print(WHEEL_DIAMETER * 0.95, 2); Serial.println(" cm");
//     // Serial.print("  Si 105 cm -> Nouveau diametre = "); Serial.print(WHEEL_DIAMETER * 1.05, 2); Serial.println(" cm");
//     // Serial.print("  Si 110 cm -> Nouveau diametre = "); Serial.print(WHEEL_DIAMETER * 1.10, 2); Serial.println(" cm");
    
//     Serial.println("\n\n=== TEST 3: VERIFICATION ROTATION ===");
//     Serial.println("Le robot va tourner de 180 degres (demi-tour)");
//     Serial.println("Verifiez si il tourne exactement 180 degres");
//     Serial.println("\nDemarrage dans 5 secondes...\n");
    
    
//     Serial.println(">>> ROTATION 180 DEGRES <<<");
    
//     movement.moveDistance(20);
//     //movement.rotate(90); // Petit mouvement pour eviter blocage
   

   
    
//     Serial.println("\n=== RESULTATS ROTATION ===");
//     Serial.println("Le robot a-t-il tourne exactement 180 degres?");
//     Serial.println("  - Si NON, il tourne TROP     -> Reduire WHEEL_BASE");
//     Serial.println("  - Si NON, il ne tourne PAS ASSEZ -> Augmenter WHEEL_BASE");
//     Serial.println("");
//     Serial.println("Formule de correction:");
//     Serial.println("  Nouveau_WHEEL_BASE = WHEEL_BASE × (Angle_reel / 180)");
//     Serial.println("");
//     Serial.println("Exemples:");
//     Serial.print("  Si 160° -> Nouveau WHEEL_BASE = "); Serial.print(WHEEL_BASE * (160.0/180.0), 2); Serial.println(" cm");
//     Serial.print("  Si 170° -> Nouveau WHEEL_BASE = "); Serial.print(WHEEL_BASE * (170.0/180.0), 2); Serial.println(" cm");
//     Serial.print("  Si 190° -> Nouveau WHEEL_BASE = "); Serial.print(WHEEL_BASE * (190.0/180.0), 2); Serial.println(" cm");
//     Serial.print("  Si 200° -> Nouveau WHEEL_BASE = "); Serial.print(WHEEL_BASE * (200.0/180.0), 2); Serial.println(" cm");
    
    
//     Serial.println("\n\n=========================================");
//     // Serial.println("  CALIBRATION TERMINEE");
//     // Serial.println("=========================================");
//     // Serial.println("\nModifiez include/settings.h avec les nouvelles valeurs");
//     // Serial.println("puis recompilez et retestez!");
// }