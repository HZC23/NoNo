#ifndef SENSOR_TASK_H
#define SENSOR_TASK_H

#include <Arduino.h>
#include "fonctions.h"

// Function to initialize sensors
void initSensors() {
    // Initialize ultrasonic sensor
    pinMode(TRIGGER, OUTPUT);
    pinMode(ECHO, INPUT);
    // Additional sensor initialization can be added here
}

// Function to update sensor readings
void sensor_update_task() {
    // Non-blocking sensor update logic
    // Read ultrasonic sensor data
    digitalWrite(TRIGGER, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIGGER, LOW);
    
    long duration = pulseIn(ECHO, HIGH);
    float distance = duration * 0.034 / 2; // Calculate distance in cm

    // Process distance data (e.g., update state, trigger actions)
    if (distance < SOME_THRESHOLD) {
        // Handle obstacle detection
    }
    
    // Additional sensor update logic can be added here
}

#endif // SENSOR_TASK_H