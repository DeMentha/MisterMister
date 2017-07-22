/*
  Logger.cpp - Class for logging onto the hardware serial.
*/
#include "Arduino.h"
#include "Logger.h"
#include <stdio.h>

void Logger::start() {
  Serial.begin(9600);
}

void Logger::log(const char *message) {
    char output[strlen(message) + 2];
    strcpy(output, message);
    strcat(output, "\n");
    Serial.print(output);
}
