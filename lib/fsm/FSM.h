
#pragma once


enum class FsmAction {
	INIT,
	MOVE_FORWARD,
	MOVE_BACKWARD,
	TURN_AROUND,
	END,
	EMERGENCY_STOP 
};

struct FsmContext {
	FsmAction currentAction;
	unsigned long stateStartMs;
};

void fsmInit(FsmContext &ctx);

void fsmStep(FsmContext &ctx);

void fsmChangeAction(FsmContext &ctx, FsmAction next);

void fsmEmergencyStop(FsmContext &ctx);

// Déclenchement prioritaire de l'arrêt d'urgence (appelable depuis ISR/callback)
void fsmTriggerEmergency();


