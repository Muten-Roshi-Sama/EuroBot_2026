
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
    // ------------------------------------
    switch (ctx.currentAction) {

    // 1. INIT
    case FsmAction::INIT: {
      //
      if (!taskManager) {
        // create TaskManager
        taskManager = new TaskManager(&movement);
        // Add task
        taskManager->addTask(new MoveTask(50.0f, DEFAULT_SPEED, 0));   // move 50 cm
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

    // 2. IDLE
    case FsmAction::IDLE: {
      
      if (taskManager && !taskManager->isIdle()) {
        //TODO: ADD "if" starting rope pulled.

        // start 100sec timer

        ctx.currentAction = FsmAction::TASK;  // if tasks queued -> go to TASK state
        debugPrintf(DBG_FSM, "IDLE -> set TASK (current=%d)", (int)ctx.currentAction);
      } 
      break;
    }

    // 3. TASK
    case FsmAction::TASK: {
      if(taskManager) taskManager->tick(); //! runs tasks and updateISR every 100ms internally

      // Check for Interruptions every 100ms (inside taskManager)
      // if (taskManager && taskManager->isIdle()) ctx.currentAction = FsmAction::IDLE;

    }

    
    // ===========================
    case FsmAction::EMERGENCY_STOP: {
        movement.stop();
    break;
    }

    case FsmAction::TIMER_END: {
        movement.stop();
    break;
    }

    default: {
      // Safety: reset to INIT on unknown action
        fsmChangeAction(ctx, FsmAction::INIT);
      break;
    }
  
  
  }
}



