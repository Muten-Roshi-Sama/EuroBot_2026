
#include <Arduino.h>
#include "settings.h"
#include "globals.h"
#include "FSM.h"
// Libs
#include "../task_manager/TaskManager.h"
#include "../drivers/button/Button.h"
#include "../movement/Movement.h"
#include "../tasks/MoveTask.h"
#include "../tasks/GyroMoveTask.h"
#include "../util/Debug.h"



// Instance du système de mouvement
Movement movement;
Button emergencyBtn(EMERGENCY_PIN, true, 2);
static void markStateStart(FsmContext &ctx);


// =================================

void fsmInit(FsmContext &ctx) {
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
  ctx.currentAction = next;
  markStateStart(ctx);
  // debugPrintf(DBG_FSM, "FSM -> %d", (int)next);
}

// =============================



void fsmStep(FsmContext &ctx) {
    // ------------------------------------
    switch (ctx.currentAction) {

    // 1. INIT
    case FsmAction::INIT: {

      movement.begin(WHEEL_DIAMETER, WHEEL_BASE, ENCODER_RESOLUTION, 
                      ENCODER_PIN_LEFT, ENCODER_PIN_RIGHT, DEFAULT_SPEED);
      delay(200);



      // create TaskManager
      if (!taskManager) taskManager = new TaskManager(&movement);

      // Add task ONCE
      static bool tasksEnqueued = false;
      if (!tasksEnqueued) {

        taskManager -> addTask(new GyroMoveTask(150.0f, 110.0f, 0));
         // move forward 100 cm
        // taskManager -> addTask(new GyroMoveTask(300.0f, 50, 0)); // move forward 100 cm
        delay(100);
        // taskManager -> addTask(new RotateTask(90.0f, DEFAULT_SPEED, 0)); // rotate +90 degrees

        tasksEnqueued = true;
      }
      
      //* Initialize all sensors
      

      
      ctx.currentAction = FsmAction::IDLE;
      debugPrintf(DBG_FSM, "FSM -> Idle");
      break;
    }

    // 2. IDLE
    case FsmAction::IDLE: {
      
      if (taskManager && !taskManager->isIdle()) {
        //TODO: ADD "if" starting rope pulled.

        // start 100sec timer

        ctx.currentAction = FsmAction::TASK;  // if tasks queued -> go to TASK state
        debugPrintf(DBG_FSM, "FSM -> Task");
      } 
      break;
    }

    // 3. TASK
    case FsmAction::TASK: {
      if(taskManager) taskManager->tick(); //! runs tasks and updateISR every 100ms internally

      // Check for Interruptions every 100ms
      if (taskManager && taskManager->isIdle()) ctx.currentAction = FsmAction::IDLE;
      break;
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

    case FsmAction::motorTest: {
        runMotorEncoderDiagnostics(movement);
        // ctx.currentAction = FsmAction::IDLE;  
        break;
    }

    default: {
      // Safety: reset to INIT on unknown action
        fsmChangeAction(ctx, FsmAction::INIT);
      break;
    }
  
  
  }
}


// Diagnostic test: run both motors forward/backward and print encoder ticks.
// Put this in a temporary test routine and call it from setup() once.

void runMotorEncoderDiagnostics(Movement &mv) {
    Serial.println(F("DIAG: reset encoders"));
    mv.resetEncoders();
    delay(200);

    Serial.println(F("DIAG: both motors FORWARD @150 for 1500ms"));
    mv.motorLeft->setSpeed(150);
    mv.motorRight->setSpeed(150);
    mv.motorLeft->run(FORWARD);
    mv.motorRight->run(FORWARD);
    delay(1500);
    noInterrupts();
    long L1 = mv.getLeftTicks();
    long R1 = mv.getRightTicks();
    interrupts();
    Serial.print(F("DIAG: forward -> L=")); Serial.print(L1); Serial.print(F("  R=")); Serial.println(R1);

    Serial.println(F("DIAG: both motors BACKWARD @150 for 1500ms"));
    mv.resetEncoders();
    mv.motorLeft->setSpeed(150);
    mv.motorRight->setSpeed(150);
    mv.motorLeft->run(BACKWARD);
    mv.motorRight->run(BACKWARD);
    delay(1500);
    noInterrupts();
    long L2 = mv.getLeftTicks();
    long R2 = mv.getRightTicks();
    interrupts();
    Serial.print(F("DIAG: backward -> L=")); Serial.print(L2); Serial.print(F("  R=")); Serial.println(R2);

    Serial.println(F("DIAG: left motor only FORWARD @150 for 1000ms"));
    mv.resetEncoders();
    mv.motorLeft->setSpeed(150);
    mv.motorLeft->run(FORWARD);
    mv.motorRight->run(RELEASE);
    delay(1000);
    noInterrupts();
    long L3 = mv.getLeftTicks();
    long R3 = mv.getRightTicks();
    interrupts();
    Serial.print(F("DIAG: left only -> L=")); Serial.print(L3); Serial.print(F("  R=")); Serial.println(R3);

    Serial.println(F("DIAG: right motor only FORWARD @150 for 1000ms"));
    mv.resetEncoders();
    mv.motorLeft->run(RELEASE);
    mv.motorRight->setSpeed(150);
    mv.motorRight->run(FORWARD);
    delay(1000);
    noInterrupts();
    long L4 = mv.getLeftTicks();
    long R4 = mv.getRightTicks();
    interrupts();
    Serial.print(F("DIAG: right only -> L=")); Serial.print(L4); Serial.print(F("  R=")); Serial.println(R4);

    mv.stop();
    Serial.println(F("DIAG: done"));
}



