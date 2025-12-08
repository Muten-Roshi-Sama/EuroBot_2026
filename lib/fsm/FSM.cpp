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
        // Si le capteur fonctionne pas, on s'arrête d'office dans cette version
        // Serial.println("Initialisation du LIDAR...");
        // if (!initLidar()) {
        //     Serial.println("ERREUR : Impossible d'initialiser le LIDAR");
        //     fsmChangeAction(ctx, FsmAction::END);
        //     break;
        // }
        // Serial.println("LIDAR OK");

        fsmChangeAction(ctx, FsmAction::MOVE1);
        break;
    }

    case FsmAction::MOVE1: {
        //movement.moveDistanceStepped(10.0); 
        fsmChangeAction(ctx, FsmAction::TURN1);  
    }

    case FsmAction::TURN1: {
        movement.rotateLeft(45);
        break;
    }

    case FsmAction::MOVE2: {
        //movement.moveDistanceStepped(20.0); 
        fsmChangeAction(ctx, FsmAction::MOVE3);  
    }

    case FsmAction::MOVE3: {
        //movement.moveDistanceStepped(-20.0); 
        fsmChangeAction(ctx, FsmAction::TURN2);  
    }

    case FsmAction::TURN2: {
        movement.rotateRight(45); 
        fsmChangeAction(ctx, FsmAction::MOVE4);  
    }

    case FsmAction::MOVE4: {
        //movement.moveDistanceStepped(40); 
        fsmChangeAction(ctx, FsmAction::TURN3);  
    }

     case FsmAction::TURN3: {
        movement.rotateLeft(45); 
        fsmChangeAction(ctx, FsmAction::MOVE5);  
    }

     case FsmAction::MOVE5: {
        //movement.moveDistanceStepped(15, 5); 
        fsmChangeAction(ctx, FsmAction::TURN4);  
    }

     case FsmAction::TURN4: {
        movement.rotateLeft(45); 
        fsmChangeAction(ctx, FsmAction::MOVE6);  
    }

     case FsmAction::MOVE6: {
        //movement.moveDistanceStepped(70); 
        fsmChangeAction(ctx, FsmAction::MOVE_UP);  
    }

    case FsmAction::MOVE_UP: {
        fsmChangeAction(ctx, FsmAction::END);
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
