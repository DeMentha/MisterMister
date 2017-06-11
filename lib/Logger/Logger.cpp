/*
  Logger.cpp - Class for logging onto the hardware serial.
*/
#include "Arduino.h"
#include "Logger.h"

void Logger::start() {
  Serial.begin(9600);
}

void Logger::log(int level, char *message) {
  if (false) {
    Serial.print(message);
  }
}
