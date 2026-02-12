
#include <Arduino.h>
#include "FSM.h"

#include "config.h"
#include "../../src/config.h"
#include "globals.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <Wire.h>

// Tsk and Drivers
// #include "../task_manager/TaskManager.h"
// #include "../drivers/launch_trigger/LaunchTrigger.h"

// Stepper and Servo

// Movement
#include "../movement/movement.h"
// utils
#include "../util/Debug.h"

// LiDAR
// #include "../detection/capteur_lidar.h"

// ==================================
//      Globals Instanciations
// ==================================
Movement movement(ENA, IN1, IN2, ENB, IN3, IN4);

// LaunchTrigger launchTrigger(LAUNCH_TRIGGER_PIN, 3);
// Button emergencyBtn(EMERGENCY_PIN, false, 1);
// static ServoController servoCtrl(SERVO_PIN);
// static StepperController stepperCtrl(STEPPER_M1_PIN1, STEPPER_M1_PIN2, STEPPER_M1_PIN3, STEPPER_M1_PIN4, STEPPER_M2_PIN1, STEPPER_M2_PIN2, STEPPER_M2_PIN3, STEPPER_M2_PIN4);

// ----------------- Helpers functions ----------------
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

void printSensorsData(const SensorsData &data) {
    Serial.println("\n========== SENSOR READINGS ==========");
    // Serial.print("IMU:      X = "); Serial.print(data.imu.ax, 2);
    // Serial.print(" | Y = "); Serial.print(data.imu.ay, 2);
    // Serial.print(" | Z = "); Serial.println(data.imu.az, 2);
    
    Serial.print("MPU Accel: ");
    Serial.print(data.mpu.ax, 2); Serial.print(" ");
    Serial.print(data.mpu.ay, 2); Serial.print(" ");
    Serial.print(data.mpu.az, 2);
    Serial.print(" | Yaw: ");
    Serial.println(data.mpu.yaw, 2);
    // Serial.println();
    
    // Ultrasonic
    // Serial.print("US Front: ");
    // Serial.print(data.usFront.distanceCm, 2); Serial.print(" cm");
    // Serial.print(" | Valid: "); Serial.println(data.usFront.valid ? "YES" : "NO");
    
    // LIDAR
    // Serial.print("LIDAR Front: ");
    // Serial.print(data.lidarFront.distanceCm, 2); Serial.print(" cm");
    // Serial.print(" | Valid: "); Serial.println(data.lidarFront.valid ? "YES" : "NO");
    
    // Encoders
    Serial.print("Encoders: ");
    Serial.print("Left = "); Serial.print(data.encoderLeft.distance_cm, 2); Serial.print(" cm");
    Serial.print(" | Right = "); Serial.print(data.encoderRight.distance_cm, 2); Serial.println(" cm");

    Serial.println("=====================================");
}

// ========== FSM Init =======================
static void markStateStart(FsmContext &ctx) {ctx.stateStartMs = millis();}
void fsmChangeAction(FsmContext &ctx, FsmAction next) {ctx.currentAction = next; markStateStart(ctx);} // debugPrintf(DBG_FSM, "FSM -> %d", (int)next); 
void fsmInitializeSystem(FsmContext &ctx)
{
  // 1. Hardware init
  // launchTrigger.begin();
  // emergencyBtn.begin();
  // stepperCtrl.begin();
  // servoCtrl.begin();
  
  // movement.begin(ENA, IN1, IN2, ENB, IN3, IN4);
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
  // delay(200);
  // debugPrintf(DBG_FSM, "1");

  // 4. Task manager
  // static TaskManager taskManagerInstance;
  // taskManager = &taskManagerInstance;
  // delay(200);
  // debugPrintf(DBG_FSM, "2");

  // 5. Match TIMER
  ctx.matchActive = false;
  ctx.matchDurationMs = MATCH_DURATION_MS;
  ctx.matchStartMs = 0;
  markStateStart(ctx);
  // debugPrintf(DBG_FSM, "FSM -> Init");
  ctx.currentAction = FsmAction::INIT;
}



// =========== FSM ==================
void fsmStep(FsmContext &ctx, const SensorsData &sensorsData)
{

  // ---- SENSOR PRINTS -----
    static unsigned long millis_print = 0;
    if(ctx.matchActive && (millis() - millis_print >= 2000)) { printSensorsData(sensorsData); millis_print = millis(); }
    
  // ------ MATCH TIMER CHECK ------
    if (ctx.matchActive && (millis() - ctx.matchStartMs >= ctx.matchDurationMs)) {
      ctx.matchActive = false;
      movement.stopMotors();
      debugPrintf(DBG_FSM, "Match timer elapsed -> TIMER_END");
      ctx.currentAction = FsmAction::TIMER_END;
  }
  


  // =======================================================
  //             FSM State Machine
  // =======================================================
  switch (ctx.currentAction){
    // =========== 1. INIT ===========
  case FsmAction::INIT:
  {
    debugPrintf(DBG_FSM, "INIT case entered");

    // printFreeMemory("After FsmAction:INIT");
    // ctx.currentAction = FsmAction::IDLE;
    // debugPrintf(DBG_FSM, "System Init done -> FSM IDLE (waiting for launch signal)");

    // MOVE COMMANDS
    // ctx.commandQueue.push({RobotCommandType::MOVE_FORWARD_CM, 30.0f});
    // ctx.commandQueue.push({RobotCommandType::ROTATE_DEG, 90.0f});

    // TUNING COMMANDS
    // ctx.commandQueue.push({RobotCommandType::TUNE_PID, (float)TUNE_DIST});
    // ctx.commandQueue.push({RobotCommandType::TUNE_PID, (float)TUNE_ANGLE});
    ctx.commandQueue.push({RobotCommandType::TUNE_PID, (float)TUNE_BOTH});

    ctx.currentAction = FsmAction::IDLE;
    break;
  }

    // =========== 2. IDLE ===============
  case FsmAction::IDLE:
  {
      // ADD LaunchTrigger HERE

      // start 100sec timer
      ctx.matchStartMs = millis();
      ctx.matchActive = true;
      ctx.matchDurationMs = MATCH_DURATION_MS;

      debugPrintf(DBG_FSM, "FSM -> DISPATCH_CMD");
      fsmChangeAction(ctx, FsmAction::DISPATCH_CMD); // if tasks queued -> go to TASK state
    break;
  }

  // =========== 3. ROBOTS COMMANDS  ============ (DISPATCH_CMD, EXEC_MOVE, EXEC_ROTATE)
  case FsmAction::DISPATCH_CMD:
  {
    // 1. Empty Queue
    if (ctx.commandQueue.empty()) { break; }

    // 2. Get next command
    ctx.currentCommand = ctx.commandQueue.front();
    ctx.commandQueue.pop();

    // 3. Dispatch to movement
    switch (ctx.currentCommand.type) {
        case  RobotCommandType::MOVE_FORWARD_CM:
            movement.startForward(ctx.currentCommand.value);
            fsmChangeAction(ctx, FsmAction::EXEC_MOVE);
            break;

        case RobotCommandType::ROTATE_DEG:
            movement.startRotate(ctx.currentCommand.value);
            fsmChangeAction(ctx, FsmAction::EXEC_ROTATE);
            break;

        case RobotCommandType::TUNE_PID:
            fsmChangeAction(ctx, FsmAction::TUNE_PID);
            break;

        // case STEPPER_UP:

        default: fsmChangeAction(ctx, FsmAction::DISPATCH_CMD); break;
    }
    break;
  }

  case FsmAction::EXEC_MOVE:
  {
    movement.update(sensorsData);
    if (movement.isDone()) { ctx.currentAction = FsmAction::DISPATCH_CMD; } // get next command
    break;
  }

  case FsmAction::EXEC_ROTATE:
  {
    movement.update(sensorsData);
    if (movement.isDone()) { ctx.currentAction = FsmAction::DISPATCH_CMD; } // get next command
    break;
  }

  case FsmAction::TUNE_PID:
  {
    static bool started = false;
    debugEnable(DBG_MOVEMENT);
    TuneMode mode = (TuneMode)((int)ctx.currentCommand.value);

    if (!started) {
      started = true;
      switch (mode) {
        case TUNE_DIST:
            movement.setDistancePID(DISTANCE_PID_DEFAULT.kp, 0.0f, DISTANCE_PID_DEFAULT.kd); // set Ki = 0
            movement.setAnglePID(0,0,0);
            movement.startForward(50); 
            break;

        case TUNE_ANGLE:
            movement.setDistancePID(0,0,0);
            movement.setAnglePID(ANGLE_PID_DEFAULT.kp, 0.0f, ANGLE_PID_DEFAULT.kd); // set Ki = 0
            movement.startForward(50);
            break;

        case TUNE_BOTH:
            movement.setDistancePID(DISTANCE_PID_DEFAULT.kp, 0.0f, DISTANCE_PID_DEFAULT.kd); // set Ki = 0
            movement.setAnglePID(ANGLE_PID_DEFAULT.kp, 0.0f, ANGLE_PID_DEFAULT.kd); // set Ki = 0
            movement.startForward(50);
            break;

        case TUNE_ROTATE:
            movement.setDistancePID(0,0,0);
            movement.setAnglePID(ANGLE_PID_DEFAULT.kp, 0.0f, ANGLE_PID_DEFAULT.kd); // set Ki = 0
            movement.startRotate(90);
            break;

        default:
            break;
    }
  }

    movement.update(sensorsData);

    if (movement.isDone()) { 
        started = false; 
        movement.setDistancePID(DISTANCE_PID_DEFAULT.kp, DISTANCE_PID_DEFAULT.ki, DISTANCE_PID_DEFAULT.kd);  // restore default PID after tuning
        movement.setAnglePID(ANGLE_PID_DEFAULT.kp, ANGLE_PID_DEFAULT.ki, ANGLE_PID_DEFAULT.kd);
        fsmChangeAction(ctx, FsmAction::DISPATCH_CMD); }

    break;
  }

    // ======== 4. EMERGENCY STOP ============
  case FsmAction::EMERGENCY_STOP:
  {
    movement.stopMotors();
    break;
  }

  case FsmAction::TIMER_END:
  {
    movement.stopMotors();
    ctx.matchActive = false;
    break;
  }

    // ============ Default ============
  default:
  {
    // Safety: reset to INIT on unknown action
    fsmChangeAction(ctx, FsmAction::EMERGENCY_STOP);
    break;
  }
  }
}


// ---------------------------- END of File -------------------------------------


    // ========= 3. TASK ============
  // case FsmAction::TASK:
  // {
  //   // Start next command if idle
  //   if (ctx.currentCommand.type == RobotCommandType::NONE && !ctx.commandQueue.empty()) {
  //       ctx.currentCommand = ctx.commandQueue.front();
  //       ctx.commandQueue.pop();

  //       // Start movement
  //       switch(ctx.currentCommand.type) {
  //           case RobotCommandType::MOVE_FORWARD_CM:
  //               movement.startForward(ctx.currentCommand.value);
  //               break;
  //           case RobotCommandType::ROTATE_DEG:
  //               movement.startRotate(ctx.currentCommand.value);
  //               break;
  //           default: break;
  //       }
  //   }

  //   // Update movement and check completion
  //   if (!movement.isDone()) {
  //       movement.update(sensorsData);
  //   } else {
  //       // Command finished
  //       ctx.currentCommand.type = RobotCommandType::NONE;
  //   }
    
  //   break;
  // }
