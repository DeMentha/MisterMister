/*
    Phase.cpp - Abstract class that defines the base implementation of a
    Phase.
*/

#include "Phase.h"
#include "MisterState.h"

void Phase::setup() {
    if (setupComplete) {
        return;
    }
    internalSetup();
    setupComplete = true;
}

MisterState Phase::check() {
    return OK;
    // TODO: for some reason calling internalCheck() breaks this and the tests
    // never finish. Need to figure out how to properly use Abstract classes.
    // return internalCheck();
}