
#pragma once


enum class FsmAction {
	INIT,
	IDLE,
	TASK,
	TIMER_END,
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



