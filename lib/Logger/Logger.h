/*
  Logger.h - Class for logging onto the hardware serial.
*/
#ifndef Logger_h
#define Logger_h

#include "Arduino.h"

class Logger {
public:
  static void start();
  static void log(const char* message);
};

#endif