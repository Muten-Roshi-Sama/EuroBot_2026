

#include "FSM.h"

void fsmInit(FsmContext &ctx) {
  ctx.currentAction = FsmAction::INIT;
}

void fsmStep(FsmContext &ctx) {
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
      // Appel à la librairie EmergencyStop
      if (emergency_button_pressed()) {
        // Ici, on peut ajouter la logique d'arrêt d'urgence (ex: couper moteurs, etc)
        // Serial.println("Arrêt d'urgence déclenché !");
      }
      break;
    default:
      ctx.currentAction = FsmAction::INIT;
      break;
  }
}
