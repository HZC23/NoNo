#include "hardware.h"
#include "robot.h"
#include "logger.h"

// Initialize sensor pins and interrupts
void sensor_init() {
  pinMode(TRIGGER, OUTPUT);
  pinMode(ECHO, INPUT);
  // The interrupt is now CHANGE, so it fires on both RISING and FALLING edges.
  attachInterrupt(digitalPinToInterrupt(ECHO), onEcho, CHANGE);
  LOG_DEBUG("Ultrasonic sensor initialized with CHANGE interrupt.");
}

// Non-blocking task to update sensor readings
void sensor_update_task(Robot& robot) {
  static unsigned long last_ping_time = 0;
  unsigned long current_time_micros = micros();

  // 1. Check if a new echo has been received from the ISR
  if (echo_received) {
    // Disable interrupts while copying ISR-shared variables to avoid torn/race reads
    noInterrupts();
    unsigned long local_echo_start = echo_start_time;
    unsigned long local_echo_end = echo_end_time;
    interrupts();
    
    echo_received = false; // Reset the flag for the next reading
    unsigned long duration = local_echo_end - local_echo_start;

    // Check for a valid duration. Anything over the timeout is invalid.
    if (duration > 0 && duration < ULTRASONIC_PULSE_TIMEOUT_US) {
      robot.dusm = duration / ULTRASONIC_DURATION_TO_CM_DIVISOR;
    } else {
      robot.dusm = ULTRASONIC_ERROR_VALUE;
    }
  }

  // 2. Trigger a new ping at a fixed interval
  // Cast to unsigned long to avoid overflow in multiplication
  if (current_time_micros - last_ping_time > ((unsigned long)ULTRASONIC_PING_INTERVAL_MS * 1000UL)) {
    last_ping_time = current_time_micros;

    // If we are still "measuring" from the last ping, it means the FALLING edge was never detected.
    // This is a timeout condition. Protect access to is_measuring.
    noInterrupts();
    if (is_measuring) {
      is_measuring = false; // Reset the measurement state
      interrupts();
      robot.dusm = ULTRASONIC_ERROR_VALUE; // Report an error
      LOG_DEBUG("Ultrasonic sensor timeout.");
    } else {
      interrupts();
    }

    // Trigger the ultrasonic pulse for a new measurement
    digitalWrite(TRIGGER, LOW);
    delayMicroseconds(ULTRASONIC_TRIGGER_PULSE_LOW_US);
    digitalWrite(TRIGGER, HIGH);
    delayMicroseconds(ULTRASONIC_TRIGGER_PULSE_HIGH_US);
    digitalWrite(TRIGGER, LOW);
  }
}
