#include <Logger.h>
#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

void Logger::start() {
  Serial.begin(9600);
}

void Logger::log(int level, char *message) {
  if (false) {
    Serial.print(message);
  }
}
