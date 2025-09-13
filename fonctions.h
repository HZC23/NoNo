#ifndef FONCTIONS_H
#define FONCTIONS_H

#include <Arduino.h>
#include "state.h"

// Prototypes for functions that remain here
void Mcap(Robot& robot, int n);
void sendPeriodicData(Robot& robot);

// Implementations
inline void Mcap(Robot& robot, int n) {
  float heading_sum = 0;
  for (int i = 0; i < n; i++) {
    // Note: getCalibratedHeading is now in compass.h, ensure it's included where Mcap is called
    // For now, assuming the main .ino includes everything needed.
    heading_sum += getCalibratedHeading(robot);
  }
  robot.cap = heading_sum / n;
}

inline void sendPeriodicData(Robot& robot) {
  if (millis() - robot.lastReportTime > robot.reportInterval) {
    robot.lastReportTime = millis();
    // Example of data to send
    // Serial.print("Cap:");
    // Serial.println(robot.cap);
  }
}

#endif // FONCTIONS_H