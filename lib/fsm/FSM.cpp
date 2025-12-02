
#include <Arduino.h>
#include "settings.h"
#include "globals.h"
#include "FSM.h"
// Libs
#include "../task_manager/TaskManager.h"
#include "../drivers/button/Button.h"
#include "../movement/Movement.h"
#include "../tasks/MoveTask.h"
#include "../util/Debug.h"



// Instance du système de mouvement
Movement movement;
Button emergencyBtn(EMERGENCY_PIN, true, 2);
static void markStateStart(FsmContext &ctx);
void calibrateEncoders();


// =================================

void fsmInit(FsmContext &ctx) {
  ctx.currentAction = FsmAction::INIT;
  markStateStart(ctx);
  // ===========

  // DEBUG prints
  debugInit(9600, DBG_FSM | DBG_TASKMANAGER);

  // Emergency Button
  emergencyBtn.begin();
  pinMode(EMERGENCY_PIN, INPUT_PULLUP);
  // attachInterrupt(digitalPinToInterrupt(EMERGENCY_PIN), emergencyISR, FALLING);

  // Others

  //============
}

static void markStateStart(FsmContext &ctx) {
  ctx.stateStartMs = millis();
}

void fsmChangeAction(FsmContext &ctx, FsmAction next) {
    debugPrintf(DBG_FSM, "FSM -> %d", (int)next);
  ctx.currentAction = next;
  markStateStart(ctx);
}

// =============================




void fsmStep(FsmContext &ctx) {
    switch (ctx.currentAction) {

    // 1. INIT
    case FsmAction::INIT: {
        // Sécurité : On s'assure que le pointeur est vide avant de créer
        if (taskManager == nullptr) {
            taskManager = new TaskManager(&movement);
        }

        // Init Movement
        movement.begin(WHEEL_DIAMETER, WHEEL_BASE, ENCODER_RESOLUTION, 
                       ENCODER_PIN_LEFT, ENCODER_PIN_RIGHT, DEFAULT_SPEED);

        // Ajout des tâches (Allocation dynamique avec 'new' -> PARFAIT)
        //taskManager->addTask(new MoveTask(50.0f, DEFAULT_SPEED, 0)); 
        movement.moveDistance(100.0f, 100); // Test initial
        delay(1000);
        
        // debugPrintf(DBG_FSM, "Init termine, passage en IDLE");
        
        // On passe à l'état suivant
        fsmChangeAction(ctx, FsmAction::INIT);
        break;
    }

    // 2. IDLE (Attente de tâches)
    case FsmAction::IDLE: {
        // Si le manager existe et qu'il n'est PAS vide (tâches en attente)
        if (taskManager && !taskManager->isIdle()) {
            // debugPrintf(DBG_FSM, "Taches detectees -> Go TASK");
            
            // CORRECTION IMPORTANTE : On va vers l'état TASK
            fsmChangeAction(ctx, FsmAction::TASK);
        }
        // Sinon, on reste ici et on attend (on pourrait ajouter un timer ici)
        break;
    }

    // 3. TASK (Exécution)
    case FsmAction::TASK: {
         // A retirer après calibration
        if (taskManager) {
            // C'est ici que la magie opère (Movement, ISR check, delete task)
            taskManager->tick(); 

            // Si tout est fini, on retourne en IDLE pour attendre les prochains ordres
            if (taskManager->isIdle()) {
                debugPrintf(DBG_FSM, "Toutes taches finies -> Retour IDLE");
                movement.stop(); // Sécurité
                fsmChangeAction(ctx, FsmAction::IDLE);
            }
        }
        break;
    }

    // ARRET D'URGENCE
    case FsmAction::EMERGENCY_STOP: {
        movement.stop();
        if (taskManager) taskManager->cancelAll(); // On vide la file
        break;
    }

    default: {
        fsmChangeAction(ctx, FsmAction::INIT);
        break;
    }
  }
}




void calibrateEncoders() {
    Serial.println("Configuration actuelle:");
    Serial.print("  - Diametre roues: "); Serial.print(WHEEL_DIAMETER); Serial.println(" cm");
    Serial.print("  - Distance entre roues: "); Serial.print(WHEEL_BASE); Serial.println(" cm");
    Serial.print("  - Resolution encodeur: "); Serial.print(ENCODER_RESOLUTION); Serial.println(" ticks/tour");
    
    // Serial.println("\n=== TEST 1: MESURE RESOLUTION ENCODEUR ===");
    // Serial.println("Instructions:");
    // Serial.println("1. Le robot va avancer pendant 5 secondes");
    // Serial.println("2. Comptez MANUELLEMENT combien de fois une roue fait un tour COMPLET");
    // Serial.println("3. Notez le nombre de ticks affiche");
    // Serial.println("\nDemarrage dans 5 secondes...\n");
    // delay(5000);
    
    // // Test de 5 secondes
    // Serial.println(">>> DEBUT DU TEST <<<");
    // movement.forward(150);
    
    // for (int i = 0; i < 50; i++) {
    //     Serial.print("Ticks gauche: ");
    //     Serial.print(movement.getLeftTicks());
    //     Serial.print(" | Ticks droite: ");
    //     Serial.println(movement.getRightTicks());
    //     delay(100);
    // }
    
    // movement.stop();
    
    // long ticksLeft = movement.getLeftTicks();
    // long ticksRight = movement.getRightTicks();
    // long avgTicks = (ticksLeft + ticksRight) / 2;
    
    // Serial.println("\n>>> FIN DU TEST <<<");
    // Serial.println("\n=== RESULTATS ===");
    // Serial.print("Total ticks gauche: "); Serial.println(ticksLeft);
    // Serial.print("Total ticks droite: "); Serial.println(ticksRight);
    // Serial.print("Moyenne: "); Serial.println(avgTicks);
    
    // Serial.println("\n=== CALCUL DE LA RESOLUTION ===");
    // Serial.println("Combien de tours COMPLETS avez-vous compte?");
    // Serial.println("Formule: Resolution = Ticks / Nombre_de_tours");
    // Serial.println("");
    // Serial.println("Exemples:");
    // Serial.print("  Si 1 tour   -> Resolution = "); Serial.print(avgTicks); Serial.println(" ticks/tour");
    // Serial.print("  Si 2 tours  -> Resolution = "); Serial.print(avgTicks / 2); Serial.println(" ticks/tour");
    // Serial.print("  Si 3 tours  -> Resolution = "); Serial.print(avgTicks / 3); Serial.println(" ticks/tour");
    // Serial.print("  Si 4 tours  -> Resolution = "); Serial.print(avgTicks / 4); Serial.println(" ticks/tour");
    // Serial.print("  Si 5 tours  -> Resolution = "); Serial.print(avgTicks / 5); Serial.println(" ticks/tour");
    
    // Serial.println("\n\n=== TEST 2: VERIFICATION DISTANCE ===");
    // Serial.println("Le robot va maintenant essayer d'avancer de 100 cm");
    // Serial.println("Mesurez la distance REELLE parcourue avec un metre");
    // Serial.println("\nDemarrage dans 5 secondes...\n");
    // delay(5000);
    
    // Serial.println(">>> AVANCE DE 100 CM <<<");
    // movement.moveDistance(100);
    
    // Serial.println("\n=== RESULTATS DISTANCE ===");
    // Serial.print("Distance demandee: 100 cm");
    // Serial.print("\nQuelle distance REELLE avez-vous mesuree? ___ cm");
    // Serial.println("\n");
    // Serial.println("Formule de correction:");
    // Serial.println("  Nouveau_Diametre = WHEEL_DIAMETER × (Distance_reelle / 100)");
    // Serial.println("");
    // Serial.println("Exemples:");
    // Serial.print("  Si 90 cm  -> Nouveau diametr_e = "); Serial.print(WHEEL_DIAMETER * 0.90, 2); Serial.println(" cm");
    // Serial.print("  Si 90 cm  -> Nouveau diametre = "); Serial.print(WHEEL_DIAMETER * 0.90, 2); Serial.println(" cm");
    // Serial.print("  Si 95 cm  -> Nouveau diametre = "); Serial.print(WHEEL_DIAMETER * 0.95, 2); Serial.println(" cm");
    // Serial.print("  Si 105 cm -> Nouveau diametre = "); Serial.print(WHEEL_DIAMETER * 1.05, 2); Serial.println(" cm");
    // Serial.print("  Si 110 cm -> Nouveau diametre = "); Serial.print(WHEEL_DIAMETER * 1.10, 2); Serial.println(" cm");
    
    Serial.println("\n\n=== TEST 3: VERIFICATION ROTATION ===");
    Serial.println("Le robot va tourner de 180 degres (demi-tour)");
    Serial.println("Verifiez si il tourne exactement 180 degres");
    Serial.println("\nDemarrage dans 5 secondes...\n");
    
    
    Serial.println(">>> ROTATION 180 DEGRES <<<");
    
    movement.moveDistance(-100);
    //movement.forward(100);
    // Serial.println("Vitesse moyenne (Ticks/loop): ");
    // Serial.println(movement.getAverageSpeedTicks());
    // delay(500);
    // movement.rotate(90);
    // delay(500);
    // movement.moveDistance(80);
    // delay(500);
    // movement.rotate(90); // Petit mouvement pour eviter blocage
    // delay(500);
    // movement.moveDistance(80);
    // delay(500);
    // movement.rotate(90);
    // delay(500);
    // movement.moveDistance(80);
     // Reset des moteurs
    // delay(500);
    // movement.rotate(90);
    // delay(500);
    // movement.moveDistance(50); // Reset des moteurs
    // delay(500);
    // movement.rotate(90);
    // delay(500);
     
    // movement.moveDistance(50); // Reset des moteurs
    // delay(500);
    // movement.rotate(90);
     
    
   

   
    delay(5000);
    
    Serial.println(">>> ROTATION 180 DEGRES <<<");
    movement.rotate(180);
    
    Serial.println("\n=== RESULTATS ROTATION ===");
    Serial.println("Le robot a-t-il tourne exactement 180 degres?");
    Serial.println("  - Si NON, il tourne TROP     -> Reduire WHEEL_BASE");
    Serial.println("  - Si NON, il ne tourne PAS ASSEZ -> Augmenter WHEEL_BASE");
    Serial.println("");
    Serial.println("Formule de correction:");
    Serial.println("  Nouveau_WHEEL_BASE = WHEEL_BASE × (Angle_reel / 180)");
    Serial.println("");
    Serial.println("Exemples:");
    Serial.print("  Si 160° -> Nouveau WHEEL_BASE = "); Serial.print(WHEEL_BASE * (160.0/180.0), 2); Serial.println(" cm");
    Serial.print("  Si 170° -> Nouveau WHEEL_BASE = "); Serial.print(WHEEL_BASE * (170.0/180.0), 2); Serial.println(" cm");
    Serial.print("  Si 190° -> Nouveau WHEEL_BASE = "); Serial.print(WHEEL_BASE * (190.0/180.0), 2); Serial.println(" cm");
    Serial.print("  Si 200° -> Nouveau WHEEL_BASE = "); Serial.print(WHEEL_BASE * (200.0/180.0), 2); Serial.println(" cm");
    
    
    Serial.println("\n\n=========================================");
    // Serial.println("  CALIBRATION TERMINEE");
    // Serial.println("=========================================");
    // Serial.println("\nModifiez include/settings.h avec les nouvelles valeurs");
    // Serial.println("puis recompilez et retestez!");
}