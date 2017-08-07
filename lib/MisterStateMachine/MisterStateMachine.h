/*
    MisterStateMachine.h - Class that manages all the business logic
    of the Mister Mister state machine.
*/
#ifndef MisterStateMachine_h
#define MisterStateMachine_h

#include "Arduino.h"
#include "Phase.h"

class MisterStateMachine {
    private:
        Phase* phases;
        int size;
    public:
        int currentPhase = 0;

        MisterStateMachine(Phase*, int size);
        ~MisterStateMachine();
        void loop();
};

#endif