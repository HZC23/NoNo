#ifndef BATTERY_UTILS_H
#define BATTERY_UTILS_H

#include <Arduino.h>
#include "robot.h"

/**
 * Reads the current battery voltage with moving-average filtering
 * to reduce noise from ADC readings.
 * 
 * @return Battery voltage in volts (float)
 */
float readBatteryVoltage();

/**
 * Converts the current battery voltage to a percentage (0-100).
 * Uses LIPO or NiMH voltage ranges based on SELECTED_BATTERY_TYPE in config.h.
 * 
 * @return Battery percentage (0-100)
 */
int readBatteryPercentage();

#endif // BATTERY_UTILS_H
