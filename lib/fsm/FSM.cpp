
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
#include "../drivers/Demarage/Demarage.h"


// Instance du système de mouvement
Movement movement;
Button emergencyBtn(EMERGENCY_PIN, true, 2);
static void markStateStart(FsmContext &ctx);
void calibrateEncoders();


// =================================

void fsmInit(FsmContext &ctx) {
  debugPrintf(DBG_FSM, "FSM INIT !");
  ctx.currentAction = FsmAction::INIT;
  markStateStart(ctx);
  // ===========

  

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
  // debugPrintf(DBG_FSM, "FSM -> %d", (int)next);
  ctx.currentAction = next;
  markStateStart(ctx);
}

// =============================




void fsmStep(FsmContext &ctx) {
    switch (ctx.currentAction) {

    // 1. INIT
    case FsmAction::INIT: {
      //
      if (!taskManager) {
        // create TaskManager
        taskManager = new TaskManager(&movement);
        // Add task
        taskManager->addTask(new MoveTask(MoveTask::MoveTaskMode::MOVE_DISTANCE, 100.0f, DEFAULT_SPEED)); // move 50 cm
        
        // taskManager->addTask(new MoveTask(90.0f, DEFAULT_SPEED, 0, true)); // rotate +90 degrees
        // taskManager.addTask()

        //* Initialize all sensors

        // Initialisation du système de mouvement avec les paramètres depuis settings.h
        movement.begin(WHEEL_DIAMETER, WHEEL_BASE, ENCODER_RESOLUTION, 
                      ENCODER_PIN_LEFT, ENCODER_PIN_RIGHT, DEFAULT_SPEED);

      }

      ctx.currentAction = FsmAction::IDLE;
      debugPrintf(DBG_FSM, "INIT -> set IDLE (current=%d)", (int)ctx.currentAction);
      break;
    }

    // 2. IDLE (Attente de tâches)
    case FsmAction::IDLE: {
      
      if (taskManager && !taskManager->isIdle()) {
        // demarage_update();
        // if(demarage_is_ready()){
          // start 100sec timer
          delay(5000); // wait 5 sec before starting tasks
          ctx.currentAction = FsmAction::TASK;  // if tasks queued -> go to TASK state
          debugPrintf(DBG_FSM, "IDLE -> set TASK (current=%d)", (int)ctx.currentAction);
        // }

        
      } 
      break;
    }

    // 3. TASK (Exécution)
    case FsmAction::TASK: {

      if(taskManager) taskManager->tick(); //! runs tasks and updateISR every 100ms internally

      // Check for Interruptions every 100ms (inside taskManager)
      // if (taskManager && taskManager->isIdle()) ctx.currentAction = FsmAction::IDLE;

            
        
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





