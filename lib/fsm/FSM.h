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
