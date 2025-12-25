#include "hardware.h"
#include "robot.h"
#include "config.h"

// Reads the raw battery voltage from the VBAT pin.
// Assumes a 2:1 voltage divider and a 3.3V ADC reference.
float readBatteryVoltage() {
    int adcValue = analogRead(VBAT);
    // The ADC reading is scaled. We need to convert it back to the real battery voltage.
    // Formula: Battery Voltage = (ADC Value / ADC Max) * ADC Reference Voltage * Divider Factor
    float batteryVoltage = (adcValue / 4095.0) * 3.3 * 2.0;
    return batteryVoltage;
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
