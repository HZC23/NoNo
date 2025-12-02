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

  // Convert voltage to percentage based on selected battery type
  float maxVoltage, minVoltage;

#if SELECTED_BATTERY_TYPE == BATTERY_TYPE_LIPO
  maxVoltage = LIPO_MAX_VOLTAGE;
  minVoltage = LIPO_MIN_VOLTAGE;
#elif SELECTED_BATTERY_TYPE == BATTERY_TYPE_NIMH
  maxVoltage = NIMH_MAX_VOLTAGE;
  minVoltage = NIMH_MIN_VOLTAGE;
#else
  // Default to LiPo if no type is selected
  maxVoltage = LIPO_MAX_VOLTAGE;
  minVoltage = LIPO_MIN_VOLTAGE;
#endif

  float percentage = ((batteryVoltage - minVoltage) / (maxVoltage - minVoltage)) * 100.0;
  
  // Constrain the value between 0 and 100
  return constrain((int)percentage, 0, 100);
}

inline bool isValidNumericInput(const char* input, int minVal, int maxVal) {
  for (unsigned int i = 0; i < strlen(input); i++) {
    if (!isdigit(input[i])) {
      return false;
    }
  }
  long val = atol(input);
  return (val >= minVal && val <= maxVal);
}

#endif // SUPPORT_H