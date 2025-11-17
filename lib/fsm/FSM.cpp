

#include "FSM.h"
#include "EmergencyStop.h"

void fsmInit(FsmContext &ctx) {
  ctx.currentAction = FsmAction::INIT;
}

void fsmStep(FsmContext &ctx) {
  // Déclenchement prioritaire de l'arrêt d'urgence
  if (emergency_button_pressed() && ctx.currentAction != FsmAction::EMERGENCY_STOP) {
    ctx.currentAction = FsmAction::EMERGENCY_STOP;
  }
  
  switch (ctx.currentAction) {
    case FsmAction::INIT:
      // TODO: initialize subsystems, wait for start condition
      break;
    case FsmAction::MOVE_FORWARD:
      // TODO: command motors to move forward
      break;
    case FsmAction::MOVE_BACKWARD:
      // TODO: command motors to move backward
      break;
    case FsmAction::TURN_AROUND:
      // TODO: rotate robot 180 degrees
      break;
    case FsmAction::END:
      // TODO: stop motors, finalize
      break;
    case FsmAction::EMERGENCY_STOP:
      Serial.println("Arrêt d'urgence déclenché !");
      // Ici, on peut ajouter la logique d'arrêt d'urgence (ex: couper moteurs, etc)
      break;
    default:
      ctx.currentAction = FsmAction::INIT;
      break;
  }
}
