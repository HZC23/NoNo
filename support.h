#ifndef SUPPORT_H
#define SUPPORT_H

#include <Arduino.h>
#include "config.h"

// Prototypes
void PhareAllume();
void PhareEteint();
int readBatteryPercentage();
bool isValidNumericInput(const String& input, int minVal, int maxVal);

// Implementations
inline void PhareAllume() {
  digitalWrite(PIN_PHARE, HIGH);
}

inline void PhareEteint() {
  digitalWrite(PIN_PHARE, LOW);
}

inline int readBatteryPercentage() {
  // Based on a voltage divider with R1=6.8k and R2=10k
  // V_pin = V_battery * (R2 / (R1 + R2))
  // V_battery = V_pin * ((R1 + R2) / R2) = V_pin * 1.68
  int sensorValue = analogRead(VBAT);
  float pinVoltage = sensorValue * (5.0 / 1023.0);
  float batteryVoltage = pinVoltage * 1.68;

  // Convert voltage to percentage (assuming 8.4V is 100% and 6.0V is 0% for a 2S LiPo)
  const float MAX_VOLTAGE = MAX_BATTERY_VOLTAGE;
  const float MIN_VOLTAGE = MIN_BATTERY_VOLTAGE;
  
  float percentage = ((batteryVoltage - MIN_VOLTAGE) / (MAX_VOLTAGE - MIN_VOLTAGE)) * 100.0;
  
  // Constrain the value between 0 and 100
  return constrain((int)percentage, 0, 100);
}

inline bool isValidNumericInput(const String& input, int minVal, int maxVal) {
  for (unsigned int i = 0; i < input.length(); i++) {
    if (!isDigit(input.charAt(i))) {
      return false;
    }
  }
  long val = input.toInt();
  return (val >= minVal && val <= maxVal);
}

#endif // SUPPORT_H