
#pragma once


// enum class FsmAction {
// 	INIT,
// 	MOVE_FORWARD,
// 	MOVE_BACKWARD,
// 	TURN_AROUND,
// 	END,
// 	EMERGENCY_STOP,
//     CALIBRATE_ENCODERS, 
//     CHECK_OBSTACLE,
//     AVOID_OBSTACLE
// };

// Attention : mesurer sur le plateau où le robot commence et adpater les dimensions
enum class FsmAction {
    INIT, 
    MOVE1, // Avancer de 10cm
    TURN1, // Tourner de 45 degré vers la gauche
    MOVE2, // Avancer de 20cm
    MOVE3, // Reculer de 20cm
    TURN2, // Tourner de 45 degré vers la droite
    MOVE4, // Avancer de 40cm
    TURN3, // Tourner de 45 degré vers la gauche
    MOVE5, // Avancer de 15cm
    TURN4, // Tourner de 45 degré vers la gauche encore
    MOVE6, // Avancer de 70cm
    MOVE_UP, // Lever le plateau
    END

};

// enum class FsmAction {
//     INIT,
//     MOVE1,
//     TURN1,
//     MOVE2,
//     BACK1,
//     TURN2,
//     MOVE3,
//     TURN3,
//     MOVE4,
//     END,
//     CALIBRATE_ENCODERS,
//     CHECK_OBSTACLE,
//     AVOID_OBSTACLE,
//     EMERGENCY_STOP
// };


struct FsmContext {
	FsmAction currentAction;
	unsigned long stateStartMs;
};

void fsmInit(FsmContext &ctx);

void fsmStep(FsmContext &ctx);

void fsmChangeAction(FsmContext &ctx, FsmAction next);

void fsmEmergencyStop(FsmContext &ctx);

void calibrateEncoders();

#ifndef FSM_H
#define FSM_H

#include <Arduino.h>

// Define robot states
enum RobotState {
    IDLE,
    SEARCH_OBJECT,
    PICK_OBJECT,
    MOVE_TO_TARGET,
    DROP_OBJECT,
    ERROR
};

class FSM {
private:
    RobotState currentState;

public:
    FSM();
    void begin();
    void update();
    RobotState getState();
    void setState(RobotState state);
};

#endif