
#include <Arduino.h>
#include "FSM.h"
#include "Movement.h"
#include "settings.h"


// Instance du système de mouvement
Movement movement;


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

void fsmEmergencyStop(FsmContext &ctx) {
  // Force transition to EMERGENCY_STOP
    fsmChangeAction(ctx, FsmAction::EMERGENCY_STOP);
}

// =============================


void fsmStep(FsmContext &ctx) {
    switch (ctx.currentAction) {
    case FsmAction::INIT: {
      // Initialisation du système de mouvement avec les paramètres depuis settings.h
    movement.begin(WHEEL_DIAMETER, WHEEL_BASE, ENCODER_RESOLUTION, 
                    ENCODER_PIN_LEFT, ENCODER_PIN_RIGHT, DEFAULT_SPEED);

    fsmChangeAction(ctx, FsmAction::MOVE_FORWARD);
    break;
    }
    case FsmAction::EMERGENCY_STOP: {
        movement.stop();
    break;
    }
    case FsmAction::MOVE_FORWARD: {
        Serial.print("Action: MOVE_FORWARD\n");
        movement.moveDistance(50);
        fsmChangeAction(ctx, FsmAction::TURN_AROUND);
    break;
    }
    case FsmAction::MOVE_BACKWARD: {
        movement.backward();
      // Transition when done:
      // fsmChangeAction(ctx, FsmAction::TURN_AROUND);
    break;
    }
    case FsmAction::TURN_AROUND: {
        Serial.print("Action: TURN_AROUND\n");
        movement.rotate(-180);        // Tourne de 180° à gauche
        fsmChangeAction(ctx, FsmAction::END);
    break;
    }
    case FsmAction::END: {
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

