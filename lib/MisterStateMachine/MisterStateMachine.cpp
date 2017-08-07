/*
    MisterStateMachine.cpp - Class that manages all the business logic
    of the Mister Mister state machine.
*/

#include "MisterStateMachine.h"
#include "MisterState.h"
#include "Phase.h"

MisterStateMachine::MisterStateMachine(Phase* array, int size) {
    phases = {};
    for (int i(0); i < size; i++) {
        phases[i] = array[i];
    }
    this->size = size;
}

MisterStateMachine::~MisterStateMachine() {
    delete[] phases;
}

void MisterStateMachine::loop() {
    if (currentPhase != -1 && currentPhase < size) {
        currentPhase++;
        Phase * phase = &phases[currentPhase];
        phase->setup();
        MisterState state = phase->check();
        // if (state == RESULT_OK) {
        //     currentPhase++;
        // } else if (state == RESULT_FAIL) {
        //     currentPhase = -1;
        // }
    } else {
        // currentPhase > phases.length means MisterStateMachine is complete.
        // Run complete steps.
    }
}