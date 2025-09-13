#ifndef SENSOR_TASK_H
#define SENSOR_TASK_H

#include <Arduino.h>
#include "fonctions.h"

// --- Task Scheduling ---
const long SENSOR_TASK_INTERVAL_US = 5000; // 200 Hz
unsigned long lastSensorTaskTime = 0;

// --- Ultrasonic Sensor (Non-Blocking) ---
volatile unsigned long echoStartTime = 0;
volatile unsigned long echoEndTime = 0;
volatile bool newEchoReceived = false;

// Interrupt Service Routine for the echo pin
// This function will be called on every change on the echo pin

#ifndef IRAM_ATTR
#define IRAM_ATTR
#endif

void IRAM_ATTR echo_isr() {
  switch (digitalRead(ECHO)) {
    case HIGH: // Echo pulse has started
      echoStartTime = micros();
      break;
    case LOW: // Echo pulse has ended
      echoEndTime = micros();
      newEchoReceived = true;
      break;
  }
}

// Function to trigger a new ultrasonic reading
void trigger_ultrasonic_read() {
  digitalWrite(TRIGGER, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGGER, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER, LOW);
}

// --- Main Sensor Task ---
// This function should be called in every loop()
void sensor_update_task() {
  unsigned long currentTime = micros();

  // Check if it's time to run the sensor task
  if (currentTime - lastSensorTaskTime >= SENSOR_TASK_INTERVAL_US) {
    lastSensorTaskTime = currentTime;

    // 1. Trigger the next ultrasonic reading
    trigger_ultrasonic_read();

    // 2. Read the compass (I2C is relatively fast)
    if (compassInitialized) {
      compass.read();
    }
  }

  // 3. Check if a new ultrasonic echo has been received
  if (newEchoReceived) {
    // Disable interrupts temporarily to safely read volatile variables
    noInterrupts();
    unsigned long duration = echoEndTime - echoStartTime;
    newEchoReceived = false;
    interrupts();

    // Calculate distance in cm (0.034 / 2 = 0.017)
    int calculated_dist = duration * 0.017;

    // Update the global distance variable with a simple low-pass filter
    // alpha = 0.7 means 70% new value, 30% old value
    dusm = (0.7 * calculated_dist) + (0.3 * dusm);

    // Clamp the value
    if (dusm > 100) {
        dusm = 100;
    }
  }
}

#endif // SENSOR_TASK_H
