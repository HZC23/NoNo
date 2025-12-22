#ifndef SUPPORT_H
#define SUPPORT_H

#include <Arduino.h>
#include "config.h"

// Prototypes
void PhareAllume();
void PhareEteint();
float readBatteryVoltage(); // New prototype
int readBatteryPercentage();

// Implementations
inline void PhareAllume() {
  digitalWrite(PIN_PHARE, HIGH);
}

inline void PhareEteint() {
  digitalWrite(PIN_PHARE, LOW);
}

inline float readBatteryVoltage() {
    // Buffer for moving average, declared static to persist across calls
    const int BATTERY_READINGS_COUNT = 10;
    static float batteryReadings[BATTERY_READINGS_COUNT] = {0};
    static int readingsIndex = 0;
    static bool readingsFilled = false;
    static float sum = 0;

    // Subtract the oldest reading from the sum
    sum -= batteryReadings[readingsIndex];

    // Read current voltage and add it to the buffer and the sum
    float currentVoltage = (analogRead(VBAT) * 3.3 / 4095.0) * ((18 +10) / 10.0);
    batteryReadings[readingsIndex] = currentVoltage;
    sum += currentVoltage;

    // Move to the next index
    readingsIndex++;
    if (readingsIndex >= BATTERY_READINGS_COUNT) {
        readingsIndex = 0;
        readingsFilled = true;
    }

    // Calculate average
    int count = readingsFilled ? BATTERY_READINGS_COUNT : readingsIndex;
    if (count == 0) return 0; // Avoid division by zero on first call
    
    return sum / count;
}

inline int readBatteryPercentage() {
  float batteryVoltage = readBatteryVoltage();

  // Convert voltage to percentage based on selected battery type
  float maxVoltage, minVoltage;

#if SELECTED_BATTERY_TYPE == BATTERY_TYPE_LIPO
  maxVoltage = LIPO_MAX_VOLTAGE;
  minVoltage = LIPO_MIN_VOLTAGE;
#elif SELECTED_BATTERY_TYPE == BATTERY_TYPE_NIMH
  maxVoltage = NIMH_MAX_VOLTAGE;
  minVoltage = NIMH_MIN_VOLTAGE;
#else
  // Default to NiMH if no type is selected or unrecognized
  maxVoltage = NIMH_MAX_VOLTAGE;
  minVoltage = NIMH_MIN_VOLTAGE;
#endif

  // Avoid division by zero
  if (maxVoltage <= minVoltage) {
    return 0;
  }

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

/**
 * @brief Converts the RobotState enum to a human-readable string.
 * @param state The RobotState to convert.
 * @return A const char* representing the state.
 */
inline const char* stateToString(RobotState state) {
    switch (state) {
        case IDLE: return "IDLE";
        case MOVING_FORWARD: return "MOVING_FORWARD";
        case MOVING_BACKWARD: return "MOVING_BACKWARD";
        case TURNING_LEFT: return "TURNING_LEFT";
        case TURNING_RIGHT: return "TURNING_RIGHT";
        case MANUAL_FORWARD: return "MANUAL_FORWARD";
        case MANUAL_BACKWARD: return "MANUAL_BACKWARD";
        case MANUAL_TURNING_LEFT: return "MANUAL_TURNING_LEFT";
        case MANUAL_TURNING_RIGHT: return "MANUAL_TURNING_RIGHT";
        case OBSTACLE_AVOIDANCE: return "OBSTACLE_AVOIDANCE";
        case WAITING_FOR_TURRET: return "WAITING_FOR_TURRET";
        case FOLLOW_HEADING: return "FOLLOW_HEADING";
        case MAINTAIN_HEADING: return "MAINTAIN_HEADING";
        case BACKING_UP_OBSTACLE: return "BACKING_UP_OBSTACLE";
        case SCANNING_FOR_PATH: return "SCANNING_FOR_PATH";
        case TURNING_TO_PATH: return "TURNING_TO_PATH";
        case SMART_TURNING: return "SMART_TURNING";
        case CALIBRATING_COMPASS: return "CALIBRATING_COMPASS";
        case SMART_AVOIDANCE: return "SMART_AVOIDANCE";
        case SENTRY_MODE: return "SENTRY_MODE";
        case CHECKING_GROUND: return "CHECKING_GROUND";
        case CLIFF_DETECTED: return "CLIFF_DETECTED";
        case ANIMATING_HEAD: return "ANIMATING_HEAD";
        case APP_CONTROL: return "APP_CONTROL";
        case EMERGENCY_EVASION: return "EMERGENCY_EVASION";
        case STUCK: return "STUCK";
        default: return "UNKNOWN";
    }
}

#endif // SUPPORT_H