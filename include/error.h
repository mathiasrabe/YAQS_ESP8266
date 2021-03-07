
#ifndef ERROR
  #define ERROR

// Import required libraries
#include <Arduino.h>
#include <LittleFS.h>

void errorLog(const char* message) {
  // Write message to FS and print it to Serial
  Serial.println(message);

  File errorFile = LittleFS.open("error.log", "a");
  
  if (!errorFile) {
    Serial.println("FATAL: Could not open error file for writing");
    errorFile.close();
    return;
  }
  errorFile.println(message);
  errorFile.close();
}

#endif
