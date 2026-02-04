
#pragma once

class Movement; // forward declare

enum class Team {
	TEAM_YELLOW = 0,
	TEAM_BLUE = 1
};


enum class FsmAction {
	INIT,
	IDLE,
	TASK,
	TIMER_END,
	END,
	EMERGENCY_STOP,
	motorTest
};

struct FsmContext {
	FsmAction currentAction;
	unsigned long stateStartMs;
	Team currentTeam;
	unsigned long matchStartMs = 0;
    unsigned long matchDurationMs = 0;
    bool matchActive = false;
};

void fsmInitializeSystem(FsmContext &ctx);

void fsmStep(FsmContext &ctx);

void fsmChangeAction(FsmContext &ctx, FsmAction next);


