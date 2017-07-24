/*
    MisterStateMachine.h - Class that manages all the business logic
    of the Mister Mister state machine.
*/
#ifndef MisterStateMachine_h
#define MisterStateMachine_h

#include "Arduino.h"

class MisterStateMachine {
        static const int RESULT_OK = 0;
        static const int RESULT_FAIL = 1;
        static const int RESULT_WAIT = 2;

    public:
        enum phase {
            one,
            two,
            three,
            halt
        };

        phase currentPhase = one;

        void loop();
        int phaseOneCheck();
};

#endif