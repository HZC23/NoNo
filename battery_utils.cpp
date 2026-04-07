#include "hardware.h"
#include "robot.h"
#include "config.h"

// Non-blocking moving average filter for battery voltage
float readBatteryVoltage() {
    const int BATTERY_READINGS_COUNT = 20;
    static float batteryReadings[BATTERY_READINGS_COUNT] = {0};
    static int readingsIndex = 0;
    static bool readingsFilled = false;
    static float sum = 0;

    // Subtract the oldest reading from the sum
    sum -= batteryReadings[readingsIndex];

    // Get a new reading
    int rawReading = analogRead(VBAT);
    // The voltage divider on the Freenove board is 10k / (18k + 10k), so the factor is (18+10)/10 = 2.8
    float currentVoltage = (rawReading * 3.3 / 4095.0) * 2.8;
    
    // Add the new reading to the buffer and the sum
    batteryReadings[readingsIndex] = currentVoltage;
    sum += currentVoltage;

    // Increment the index
    readingsIndex++;
    if (readingsIndex >= BATTERY_READINGS_COUNT) {
        readingsIndex = 0;
        readingsFilled = true;
    }

    int count = readingsFilled ? BATTERY_READINGS_COUNT : readingsIndex;
    
    return sum / count;
}

// Converts the battery voltage to a percentage.
int readBatteryPercentage() {
    float voltage = readBatteryVoltage();
    float minVoltage, maxVoltage;

    #if SELECTED_BATTERY_TYPE == BATTERY_TYPE_LIPO
        minVoltage = LIPO_MIN_VOLTAGE;
        maxVoltage = LIPO_MAX_VOLTAGE;
    #else // Default to NiMH as per config.h
        minVoltage = NIMH_MIN_VOLTAGE;
        maxVoltage = NIMH_MAX_VOLTAGE;
    #endif

    if (voltage <= minVoltage) {
        return 0;
    }
    if (voltage >= maxVoltage) {
        return 100;
    }

    int percentage = (int)(((voltage - minVoltage) / (maxVoltage - minVoltage)) * 100.0);
    return percentage;
}
