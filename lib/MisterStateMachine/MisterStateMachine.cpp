/*
    MisterStateMachine.cpp - Class that manages all the business logic
    of the Mister Mister state machine.
*/

#include "MisterStateMachine.h"

void MisterStateMachine::loop() {
    if (currentPhase == one) {
        int result = phaseOneCheck();
        if (result == RESULT_OK) {
            currentPhase = two;
        } else if (result = RESULT_FAIL) {
            currentPhase = halt;
        }
    }
}

int MisterStateMachine::phaseOneCheck() {
    return RESULT_OK;
}