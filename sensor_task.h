#ifndef SENSOR_TASK_H
#define SENSOR_TASK_H

#include <Arduino.h>
#include "config.h"
#include "state.h"

volatile unsigned long echo_start = 0;
volatile unsigned long echo_end = 0;
volatile bool echo_received = false;

void echo_isr() {
  if (digitalRead(ECHO) == HIGH) {
    echo_start = micros();
  } else {
    echo_end = micros();
    echo_received = true;
  }
}

void sensor_init() {
  pinMode(TRIGGER, OUTPUT);
  pinMode(ECHO, INPUT);
  attachInterrupt(digitalPinToInterrupt(ECHO), echo_isr, CHANGE);
}

void sensor_update_task(Robot& robot) {
  static unsigned long last_ping = 0;
  const unsigned long PING_INTERVAL = 50; // ms

  if (millis() - last_ping > PING_INTERVAL) {
    last_ping = millis();
    digitalWrite(TRIGGER, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIGGER, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIGGER, LOW);
  }

  if (echo_received) {
    noInterrupts();
    unsigned long duration = echo_end - echo_start;
    echo_received = false;
    interrupts();
    robot.dusm = duration / 58;
  }
}

#endif // SENSOR_TASK_H