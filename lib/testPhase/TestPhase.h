/*
    TestPhase.h - Class that manages all the business logic
    of the Mister Mister state machine.
*/
#ifndef TestPhase_h
#define TestPhase_h

#include "Phase.h"
#include "MisterState.h"

class TestPhase: public Phase {
    public:
        TestPhase();
        ~TestPhase();
    protected:
        void internalSetup();
        MisterState internalCheck();
        void tearDown();
};

#endif