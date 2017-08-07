/*
    Phase.h - Abstract class that defines the base implementation of a
    Phase.
*/
#ifndef Phase_h
#define Phase_h

#include "Arduino.h"
#include "MisterState.h"

class Phase {
    private:
        boolean setupComplete = false;
    protected:
        virtual void internalSetup()=0;
        virtual MisterState internalCheck()=0;
        virtual void tearDown()=0;
    public:
        void setup();
        MisterState check();
};

#endif