#include "FSM.h"
#include <Arduino.h>

FSM::FSM() : currentState(IDLE) {}

void FSM::begin() {
    currentState = IDLE;
    Serial.println("FSM initialized. State: IDLE");
}

void FSM::update() {
    switch (currentState) {
        case IDLE:
            // Example logic: start searching
            Serial.println("State: IDLE -> SEARCH_OBJECT");
            currentState = SEARCH_OBJECT;
            break;

        case SEARCH_OBJECT:
            // TODO: implement object detection
            Serial.println("State: SEARCH_OBJECT -> PICK_OBJECT");
            currentState = PICK_OBJECT;
            break;

        case PICK_OBJECT:
            // TODO: implement grab logic
            Serial.println("State: PICK_OBJECT -> MOVE_TO_TARGET");
            currentState = MOVE_TO_TARGET;
            break;

        case MOVE_TO_TARGET:
            // TODO: implement movement logic
            Serial.println("State: MOVE_TO_TARGET -> DROP_OBJECT");
            currentState = DROP_OBJECT;
            break;

        case DROP_OBJECT:
            // TODO: implement drop logic
            Serial.println("State: DROP_OBJECT -> IDLE");
            currentState = IDLE;
            break;

        case ERROR:
            // Handle errors
            Serial.println("State: ERROR - check sensors and restart");
            break;
    }
}

RobotState FSM::getState() {
    return currentState;
}

void FSM::setState(RobotState state) {
    currentState = state;
}
