
#include <Arduino.h>
#include "FSM.h"

#include "config.h"
#include "globals.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <Wire.h>

// Tsk and Drivers
#include "../task_manager/TaskManager.h"
// #include "../drivers/launch_trigger/LaunchTrigger.h"

// Stepper and Servo

// Movement
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


static constexpr unsigned long MATCH_DURATION_MS = 20000; // adjust (e.g., 90000 for 90s)
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
  // 1. Hardware init
  printFreeMemory("Before HW init");
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
  // delay(200);
  // debugPrintf(DBG_FSM, "1");

  // 4. Task manager
  // static TaskManager taskManagerInstance;
  // taskManager = &taskManagerInstance;
  // delay(200);
  // debugPrintf(DBG_FSM, "2");

  // 5. FSM context init
  ctx.currentAction = FsmAction::INIT;
  ctx.matchActive = false;
  ctx.matchDurationMs = MATCH_DURATION_MS;
  ctx.matchStartMs = 0;
  markStateStart(ctx);
  // debugPrintf(DBG_FSM, "FSM -> Init");
  printFreeMemory("After TaskManager init");
}


void fsmChangeAction(FsmContext &ctx, FsmAction next) {ctx.currentAction = next; markStateStart(ctx);} // debugPrintf(DBG_FSM, "FSM -> %d", (int)next); 

// =========== FSM ==================
void fsmStep(FsmContext &ctx, const SensorsData &sensorsData)
{

  // ---- SENSOR PRINTS -----
    static unsigned long millis_print = 0;
    if(ctx.matchActive && (millis() - millis_print >= 2000)) {
      millis_print = millis();
      // ----- IMU -----
      Serial.println("\n========== SENSOR READINGS ==========");
      
      // // IMU
      // Serial.print("IMU:      X = "); Serial.print(sensorsData.imu.ax, 2);
      // Serial.print(" | Y = "); Serial.print(sensorsData.imu.ay, 2);
      // Serial.print(" | Z = "); Serial.print(sensorsData.imu.az, 2);
      // Serial.println();
      //! MPU 
      Serial.print("MPU Accel: ");
      Serial.print(sensorsData.mpu.ax, 2); Serial.print(" ");
      Serial.print(sensorsData.mpu.ay, 2); Serial.print(" ");
      Serial.println(sensorsData.mpu.az, 2);

      Serial.print("MPU Yaw: ");
      Serial.println(sensorsData.mpu.yaw, 2);
      Serial.println();
      
      // Ultrasonic
      Serial.print("US Front: ");
      Serial.print(sensorsData.usFront.distanceCm, 2); Serial.print(" cm");
      Serial.print(" | Valid: "); Serial.println(sensorsData.usFront.valid ? "YES" : "NO");
      
      // LIDAR
      Serial.print("LIDAR Front: ");
      Serial.print(sensorsData.lidarFront.distanceCm, 2); Serial.print(" cm");
      Serial.print(" | Valid: "); Serial.println(sensorsData.lidarFront.valid ? "YES" : "NO");
      
      // Encoders
      Serial.print("Encoders: ");
      Serial.print("Left = "); Serial.print(sensorsData.encoderLeft.distance_cm, 2); Serial.print(" cm");
      Serial.print(" | Right = "); Serial.print(sensorsData.encoderRight.distance_cm, 2); Serial.print(" cm");
      Serial.println();


      Serial.println("=====================================");
    }
    
  
    
  
  
  switch (ctx.currentAction){
    // 1. INIT
    // ===========================
  case FsmAction::INIT:
  {
    debugPrintf(DBG_FSM, "INIT case entered");

      printFreeMemory("After FsmAction:INIT");
      // ctx.currentAction = FsmAction::IDLE;
      ctx.currentAction = FsmAction::IDLE;
      debugPrintf(DBG_FSM, "System Init done -> FSM IDLE (waiting for launch signal)");
    // }
    
    break;
  }

    // 2. IDLE
    // ===========================
  case FsmAction::IDLE:
  {
    // launchTrigger.update();
    static unsigned long millis_print = 0;

    // if (launchTrigger.isTriggered())
    // {
    //   // launchTrigger.reset();  // Optionnal ?

    //   // start 100sec timer
      ctx.matchStartMs = millis();
      ctx.matchActive = true;
      ctx.matchDurationMs = MATCH_DURATION_MS;

      debugPrintf(DBG_FSM, "FSM -> Task");
      ctx.currentAction = FsmAction::TASK; // if tasks queued -> go to TASK state
      
    // }
    // else {
      // if(millis() - millis_print >= 2000) {
      //   millis_print = millis();
      //   debugPrintf(DBG_FSM, "Waiting for launch...");
      //   printFreeMemory("FsmAction:IDLE");
      // }
    // }
    break;
  }

    // 3. TASK
    // ===========================
  case FsmAction::TASK:
  {

    if (ctx.matchActive && (millis() - ctx.matchStartMs >= ctx.matchDurationMs))
    {
      ctx.matchActive = false;
      // movement.stop();
      debugPrintf(DBG_FSM, "Match timer elapsed -> TIMER_END");
      ctx.currentAction = FsmAction::TIMER_END;
      break;
    }

    


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


  default:
  {
    // Safety: reset to INIT on unknown action
    fsmChangeAction(ctx, FsmAction::INIT);
    break;
  }
  }
}
