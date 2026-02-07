
#include <Arduino.h>
#include "settings.h"
#include "globals.h"
#include "FSM.h"
// Tsk and Drivers
#include "../task_manager/TaskManager.h"
// #include "../drivers/launch_trigger/LaunchTrigger.h"
// #include "../drivers/button/Button.h"

// Stepper and Servo
// #include "../tasks/StepperTask.h"
// #include "../drivers/stepper_controller/StepperController.h"
// #include "../tasks/ServoTask.h"
// #include "../drivers/servo_controller/ServoController.h"


// Movement
// #include "../movement/Movement.h"
// #include "../tasks/MoveTask.h"
// #include "../tasks/GyroMoveTask.h"

// utils
#include "../util/Debug.h"

// LiDAR
// #include "../detection/capteur_lidar.h"

// ==================================
//      Globals Instanciations
// ==================================
// Movement movement;
// LaunchTrigger launchTrigger(LAUNCH_TRIGGER_PIN, 3);
// Button emergencyBtn(EMERGENCY_PIN, false, 1);

// static ServoController servoCtrl(SERVO_PIN);
// static StepperController stepperCtrl(STEPPER_M1_PIN1, STEPPER_M1_PIN2, STEPPER_M1_PIN3, STEPPER_M1_PIN4, STEPPER_M2_PIN1, STEPPER_M2_PIN2, STEPPER_M2_PIN3, STEPPER_M2_PIN4);


static constexpr unsigned long MATCH_DURATION_MS = 10000; // adjust (e.g., 90000 for 90s)
static void markStateStart(FsmContext &ctx);




void printFreeMemory(const char* label) {
  #if defined(ESP32)
    Serial.printf("[MEM] %s: Free heap: %u bytes\n", label, ESP.getFreeHeap());
  #elif defined(ARDUINO_ARCH_AVR)
    extern int __heap_start, *__brkval;
    int v;
    Serial.print("[MEM] ");
    Serial.print(label);
    Serial.print(": Free RAM: ");
    Serial.println((int)&v - (__brkval == 0 ? (int)&__heap_start : (int)__brkval));
  #endif
}



// ========== FSM Init =======================
static void markStateStart(FsmContext &ctx) {ctx.stateStartMs = millis();}

void fsmInitializeSystem(FsmContext &ctx)
{
  printFreeMemory("Before HW init");
  // 1. Hardware init
  // launchTrigger.begin();
  // emergencyBtn.begin();
  // stepperCtrl.begin();
  // servoCtrl.begin();
  // // if (!initLidar()) {
  // //   debugPrintf(DBG_FSM, "ERREUR: LIDAR non détecté !");
  // //   // Le robot peut continuer sans LIDAR ou s'arrêter selon votre choix
  // // } else {
  // //   debugPrintf(DBG_FSM, "LIDAR OK");
  // // }

  // 2. Team selection
  // pinMode(TEAM_SWITCH_PIN, INPUT_PULLUP);
  // bool teamSwitchRaw = digitalRead(TEAM_SWITCH_PIN);
  // ctx.currentTeam = (teamSwitchRaw == HIGH) ? Team::TEAM_BLUE : Team::TEAM_YELLOW;
  // delay(200);
  // debugPrintf(DBG_FSM, "Team switch read: TEAM_%c", (ctx.currentTeam == Team::TEAM_YELLOW) ? 'Y' : 'B');
  // printFreeMemory("After team select");

  // 3. Movement init
  // movement.begin(WHEEL_DIAMETER, WHEEL_BASE, ENCODER_RESOLUTION, ENCODER_PIN_LEFT, ENCODER_PIN_RIGHT, DEFAULT_SPEED);
  delay(200);
  debugPrintf(DBG_FSM, "1");

  // 4. Task manager
  static TaskManager taskManagerInstance;
  taskManager = &taskManagerInstance;
  delay(200);
  debugPrintf(DBG_FSM, "2");

  // 5. FSM context init
  ctx.currentAction = FsmAction::INIT;
  ctx.matchActive = false;
  ctx.matchDurationMs = MATCH_DURATION_MS;
  ctx.matchStartMs = 0;
  markStateStart(ctx);
  debugPrintf(DBG_FSM, "FSM -> Init");
  printFreeMemory("After TaskManager init");
}


void fsmChangeAction(FsmContext &ctx, FsmAction next) {ctx.currentAction = next; markStateStart(ctx);
    // debugPrintf(DBG_FSM, "FSM -> %d", (int)next);
  }

// =========== FSM ==================
void fsmStep(FsmContext &ctx)
{
  // ------------------------------------
  switch (ctx.currentAction){
    // 1. INIT
    // ===========================
  case FsmAction::INIT:
  {
    debugPrintf(DBG_FSM, "INIT case entered");
    //

    if (taskManager)
      {
      // ADD TASKS
      static bool tasksEnqueued = false;
      if (!tasksEnqueued) {
        /* EXAMPLE TASKS :
                        - GyroMove    : taskManager->addTask(new GyroMoveTask(300.0f, 160, 0));
                        - GyroRotate  : taskManager->addTask(new RotateGyroTask(90.0f, 150, 2.0f, 4000));

                        - StepperUp   : taskManager->addTask(new StepperUpTask(&stepperCtrl, 5000));
                        - StepperDown : taskManager->addTask(new StepperDownTask(&stepperCtrl, 5000));

                        - ServoMove   : taskManager->addTask(new ServoTask(&servoCtrl, 90, 1000));  // angle, delayMs
        */

        // CHANGE TASK BASED ON TEAM
        // if (ctx.currentTeam == Team::TEAM_YELLOW)
        // {
        //   taskManager->addTask(new GyroMoveTask(105.0f, 120, 10.0f, 1000)); // test100cm:3900, ayoub: 2800
        // }
        // else
        // {
        //   taskManager->addTask(new RotateGyroTask(20.0f, 150, 5.0f, 800)); // angle, speed, tolerance, timeout.... test100cm: 800timeout
        //   delay(200);
        //   // taskManager->addTask(new RotateGyroTask(20.0f, 150, 5.0f, 800)); // angle, speed, tolerance, timeout.... test100cm: 800timeout
        //   taskManager->addTask(new GyroMoveTask(105.0f, 120, 10.0f, 1000)); // test100cm:3900, ayoub: 2800

        // }

        if (true) {
          // taskManager->addTask(new GyroMoveTask(80.0f, 120, 10.0f, 1000));
          delay(200);
        
          // taskManager->addTask(new RotateGyroTask(20.0f, 150, 5.0f, 800));
          // delay(200);
        }

        tasksEnqueued = true;
      }

      printFreeMemory("After FsmAction:INIT");
      // ctx.currentAction = FsmAction::IDLE;
      ctx.currentAction = FsmAction::IDLE;
      debugPrintf(DBG_FSM, "System Init done -> FSM IDLE (waiting for launch signal)");
    }
    
    break;
  }

    // 2. IDLE
    // ===========================
  case FsmAction::IDLE:
  {
    //
    // launchTrigger.update();
    static unsigned long millis_print = 0;

    if (false){}
    // if (launchTrigger.isTriggered())
    // {
    //   // launchTrigger.reset();  // Optionnal ?

    //   // start 100sec timer
    //   ctx.matchStartMs = millis();
    //   ctx.matchActive = true;
    //   ctx.matchDurationMs = MATCH_DURATION_MS;

    //   ctx.currentAction = FsmAction::TASK; // if tasks queued -> go to TASK state
    //   debugPrintf(DBG_FSM, "FSM -> Task");
    // }
    else {
      if(millis() - millis_print >= 2000) {
        millis_print = millis();
        debugPrintf(DBG_FSM, "Waiting for launch...");
        printFreeMemory("FsmAction:IDLE");
      }
    }

    break;
  }

    // 3. TASK
    // ===========================
  case FsmAction::TASK:
  {
    // if (ctx.matchActive && (millis() - ctx.matchStartMs >= ctx.matchDurationMs))
    // {
    //   ctx.matchActive = false;
    //   movement.stop();
    //   ctx.currentAction = FsmAction::TIMER_END;
    //   debugPrintf(DBG_FSM, "Match timer elapsed -> TIMER_END");
    //   break;
    // }

    // if (taskManager) taskManager->tick(); //! runs tasks and updateISR every 100ms internally

    // Check for Interruptions every 100ms
    // if (taskManager && taskManager->isIdle())
    //   ctx.currentAction = FsmAction::IDLE;
    break;
  }

    // 4. EMERGENCY STOP
    // ===========================
  case FsmAction::EMERGENCY_STOP:
  {
    // movement.stop();
    break;
  }

  case FsmAction::TIMER_END:
  {
    ctx.matchActive = false;
    // movement.stop();
    break;
  }
    

  case FsmAction::motorTest:
  {
      // Serial.println("motorTest: Drawing a square");
      
      // // Configuration variables
      // const float moveDist = 50.0f;      // Distance per side (cm)
      // const int moveSpeed = 120;          // Movement speed (0-255)
      // const float moveTimeout = 3000;     // Move timeout (ms)

      // const float rotAngle = 20.0f;       // 90° turn for square corners
      // const int rotSpeed = 150;           // Rotation speed (0-255)
      // const float rotTolerance = 5.0f;    // Rotation tolerance (degrees)
      // const float rotTimeout = 850;      // Rotation timeout (ms)

      // const int delayBetweenTasks = 500;  // Delay between tasks (ms)
    

      // // Loop 4 times (4 sides + 4 turns = 8 iterations)
      // for (int side = 1; side <= 2; side++) {
      //     // Move forward
      //     Serial.print("Side ");
      //     Serial.print(side);
      //     Serial.println(": Moving forward");
      //     GyroMoveTask moveTask(moveDist, moveSpeed, 10.0f, moveTimeout);
      //     moveTask.start(movement);
      //     unsigned long moveStart = millis();
      //     while (!moveTask.isFinished() && (millis() - moveStart < moveTimeout + 500)) {
      //         moveTask.update(movement);
      //         delay(10);
      //     }
      //     movement.stop();
      //     Serial.print("Side ");
      //     Serial.print(side);
      //     Serial.println(" done");
      //     delay(delayBetweenTasks);
          
      //     // Rotate 90° (skip after last side to complete square)
      //     if (side < 4) {
      //         Serial.print("Turn ");
      //         Serial.print(side);
      //         Serial.println(": Rotating 90 degrees");
      //         RotateGyroTask rotTask(rotAngle, rotSpeed, rotTolerance, rotTimeout);
      //         rotTask.start(movement);
      //         unsigned long rotStart = millis();
      //         while (!rotTask.isFinished() && (millis() - rotStart < rotTimeout + 500)) {
      //             rotTask.update(movement);
      //             delay(10);
      //         }
      //         movement.stop();
      //         Serial.print("Turn ");
      //         Serial.print(side);
      //         Serial.println(" done");
      //         delay(delayBetweenTasks);
      //     }
      // }
      
      // movement.stop();
      // ctx.currentAction = FsmAction::TIMER_END;
      // Serial.println("Square complete!");
      // debugPrintf(DBG_FSM, "motorTest complete -> TIMER_END");
      break;
  }




  default:
  {
    // Safety: reset to INIT on unknown action
    fsmChangeAction(ctx, FsmAction::INIT);
    break;
  }
  }
}
