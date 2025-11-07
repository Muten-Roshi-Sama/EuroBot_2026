
#ifdef ARDUINO
#include <Arduino.h>
#else
#include <stdint.h>
static inline unsigned long millis() { return 0; }
#endif
#include "FSM.h"

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

// emergency flag (set by fsmTriggerEmergency)
static volatile bool gEmergencyFlag = false;

void fsmStep(FsmContext &ctx) {
  // Priority: emergency stop preempts any state
  if (gEmergencyFlag && ctx.currentAction != FsmAction::EMERGENCY_STOP) {
    fsmEmergencyStop(ctx);
    // Once we force the stop, clear the flag so it doesn't retrigger endlessly
    gEmergencyFlag = false;
    return;
  }

  switch (ctx.currentAction) {
    case FsmAction::INIT: {
      // TODO: initialize subsystems, wait for start condition
      // Example transition placeholder:
      // if (startCondition) fsmChangeAction(ctx, FsmAction::MOVE_FORWARD);
      break;
    }
    case FsmAction::EMERGENCY_STOP: {
      // TODO: immediate stop of all actuators; keep safe state
      // Remain here until reset condition
      break;
    }
    case FsmAction::MOVE_FORWARD: {
      // TODO: command motors to move forward
      // Transition when done:
      // fsmChangeAction(ctx, FsmAction::MOVE_BACKWARD);
      break;
    }
    case FsmAction::MOVE_BACKWARD: {
      // TODO: command motors to move backward
      // Transition when done:
      // fsmChangeAction(ctx, FsmAction::TURN_AROUND);
      break;
    }
    case FsmAction::TURN_AROUND: {
      // TODO: rotate robot 180 degrees
      // Transition when done:
      // fsmChangeAction(ctx, FsmAction::END);
      break;
    }
    case FsmAction::END: {
      // TODO: stop motors, finalize
      // Remain in END or reset to INIT as needed
      break;
    }
    default: {
      // Safety: reset to INIT on unknown action
      fsmChangeAction(ctx, FsmAction::INIT);
      break;
    }
  }
}

void fsmEmergencyStop(FsmContext &ctx) {
  // Force transition to EMERGENCY_STOP
  fsmChangeAction(ctx, FsmAction::EMERGENCY_STOP);
}

void fsmTriggerEmergency() {
  gEmergencyFlag = true;
}
