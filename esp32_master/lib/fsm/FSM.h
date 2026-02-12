
#pragma once

#include <queue>
#include "globals.h"

enum class Team { TEAM_YELLOW = 0, TEAM_BLUE = 1 };

// PID Tuning
enum TuneMode {
	TUNE_IDLE,
	TUNE_DIST,
	TUNE_ANGLE,
	TUNE_BOTH,	// Tune both DIST and ANG PID going forward 100cm
	TUNE_ROTATE	// Tune both DIST and ANG PID while rotating
};

//ROBOT COMMANDS
enum class RobotCommandType {
    NONE,
	TUNE_PID,
    MOVE_FORWARD_CM,
    ROTATE_DEG
};
struct RobotCommand {
    RobotCommandType type;
    float value;   // distance in cm or angle in degrees
};




enum class FsmAction {
	INIT,
	IDLE,
	DISPATCH_CMD,
	EXEC_MOVE,
	EXEC_ROTATE,
	TASK,
	TUNE_PID,
	TIMER_END,
	EMERGENCY_STOP,
};

struct FsmContext {
	Team currentTeam;
	FsmAction currentAction;
	
	bool matchActive = false;
	unsigned long stateStartMs, matchStartMs, matchDurationMs;
    
	 // Queue of robot commands
    std::queue<RobotCommand> commandQueue;
    RobotCommand currentCommand; // Optionally keep a pointer to the current command

};

void fsmInitializeSystem(FsmContext &ctx);

void fsmStep(FsmContext &ctx, const SensorsData &sensorsData);

void fsmChangeAction(FsmContext &ctx, FsmAction next);


