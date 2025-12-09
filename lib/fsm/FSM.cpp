
#include <Arduino.h>
#include "settings.h"
#include "globals.h"
#include "FSM.h"
// Tsk and Drivers
#include "../task_manager/TaskManager.h"
#include "../drivers/launch_trigger/LaunchTrigger.h"
#include "../drivers/button/Button.h"

// Movement
#include "../movement/Movement.h"
#include "../tasks/MoveTask.h"
#include "../tasks/GyroMoveTask.h"

// utils
#include "../util/Debug.h"

// LiDAR
#include "../detection/capteur_lidar.h"

// Instanciation of internal Globals
Movement movement;
LaunchTrigger launchTrigger(LAUNCH_TRIGGER_PIN, 3);
Button emergencyBtn(EMERGENCY_PIN, false, 1);
static constexpr unsigned long MATCH_DURATION_MS = 10000; // adjust (e.g., 90000 for 90s)
static void markStateStart(FsmContext &ctx);

// ========== FSM Init =======================
void fsmInitializeSystem(FsmContext &ctx)
{
  // 1. Hardware init
  launchTrigger.begin();
  emergencyBtn.begin();
  // pinMode(EMERGENCY_PIN, INPUT_PULLUP); // attachInterrupt(digitalPinToInterrupt(EMERGENCY_PIN), emergencyISR, FALLING);
  // if (!initLidar()) {
  //   debugPrintf(DBG_FSM, "ERREUR: LIDAR non détecté !");
  //   // Le robot peut continuer sans LIDAR ou s'arrêter selon votre choix
  // } else {
  //   debugPrintf(DBG_FSM, "LIDAR OK");
  // }

  // 2. Team selection
  pinMode(TEAM_SWITCH_PIN, INPUT_PULLUP);
  bool teamSwitchRaw = digitalRead(TEAM_SWITCH_PIN);
  ctx.currentTeam = (teamSwitchRaw == HIGH) ? Team::TEAM_BLUE : Team::TEAM_YELLOW;
  delay(200);
  debugPrintf(DBG_FSM, "Team switch read: TEAM_%c", (ctx.currentTeam == Team::TEAM_YELLOW) ? 'Y' : 'B');

  // 3. Movement init
  movement.begin(WHEEL_DIAMETER, WHEEL_BASE, ENCODER_RESOLUTION, ENCODER_PIN_LEFT, ENCODER_PIN_RIGHT, DEFAULT_SPEED);
  delay(200);
  debugPrintf(DBG_FSM, "1");

  // 4. Task manager
  // if (!taskManager) taskManager = new TaskManager(&movement);
  static TaskManager taskManagerInstance(&movement);
  taskManager = &taskManagerInstance;
  delay(200);
  debugPrintf(DBG_FSM, "2");

  // extern char *__brkval;
  // Serial.print("Free RAM: ");
  // Serial.println((int)SP - (int)__brkval);

  // 5. FSM context init
  ctx.currentAction = FsmAction::INIT;
  ctx.matchActive = false;
  ctx.matchDurationMs = MATCH_DURATION_MS;
  ctx.matchStartMs = 0;
  markStateStart(ctx);
  debugPrintf(DBG_FSM, "FSM -> Init");
}

static void markStateStart(FsmContext &ctx)
{
  ctx.stateStartMs = millis();
}
void fsmChangeAction(FsmContext &ctx, FsmAction next)
{
  ctx.currentAction = next;
  markStateStart(ctx);
  // debugPrintf(DBG_FSM, "FSM -> %d", (int)next);
}

// =========== FSM ==================
void fsmStep(FsmContext &ctx)
{
  // ------------------------------------
  switch (ctx.currentAction)
  {

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
        // taskManager->addTask(new GyroMoveTask(300.0f, 160, 0));
        // taskManager->addTask(new RotateGyroTask(90.0f, 150, 2.0f, 4000));

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
          taskManager->addTask(new GyroMoveTask(80.0f, 120, 10.0f, 1000));
          delay(200);
          taskManager->addTask(new RotateGyroTask(20.0f, 150, 5.0f, 800));
          delay(200);

          taskManager->addTask(new GyroMoveTask(80.0f, 120, 10.0f, 1000));
          delay(200);
          taskManager->addTask(new RotateGyroTask(20.0f, 150, 5.0f, 800));
          delay(200);


        }

        tasksEnqueued = true;
      }


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
    launchTrigger.update();

    if (launchTrigger.isTriggered())
    {
      // launchTrigger.reset();  // Optionnal ?

      // start 100sec timer
      ctx.matchStartMs = millis();
      ctx.matchActive = true;
      ctx.matchDurationMs = MATCH_DURATION_MS;

      ctx.currentAction = FsmAction::TASK; // if tasks queued -> go to TASK state
      debugPrintf(DBG_FSM, "FSM -> Task");
    }
    break;
  }

    // 3. TASK
    // ===========================
  case FsmAction::TASK:
  {
    if (ctx.matchActive && (millis() - ctx.matchStartMs >= ctx.matchDurationMs))
    {
      ctx.matchActive = false;
      movement.stop();
      ctx.currentAction = FsmAction::TIMER_END;
      debugPrintf(DBG_FSM, "Match timer elapsed -> TIMER_END");
      break;
    }




    if (taskManager) taskManager->tick(); //! runs tasks and updateISR every 100ms internally




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
    movement.stop();
    break;
  }
// runMotorEncoderDiagnostics(movement);
    

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

// Diagnostic test: run both motors forward/backward and print encoder ticks.
// Put this in a temporary test routine and call it from setup() once.

void runMotorEncoderDiagnostics(Movement &mv)
{
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
  Serial.print(F("DIAG: forward -> L="));
  Serial.print(L1);
  Serial.print(F("  R="));
  Serial.println(R1);

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
  Serial.print(F("DIAG: backward -> L="));
  Serial.print(L2);
  Serial.print(F("  R="));
  Serial.println(R2);

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
  Serial.print(F("DIAG: left only -> L="));
  Serial.print(L3);
  Serial.print(F("  R="));
  Serial.println(R3);

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
  Serial.print(F("DIAG: right only -> L="));
  Serial.print(L4);
  Serial.print(F("  R="));
  Serial.println(R4);

  mv.stop();
  Serial.println(F("DIAG: done"));
}
