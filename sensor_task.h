#ifndef SENSOR_TASK_H
#define SENSOR_TASK_H

#include <Arduino.h>
#include "config.h"
#include "state.h"

// --- État interne du module capteur ---

// Variables pour la routine de service d'interruption (ISR)
static volatile unsigned long g_echo_start_time = 0;
static volatile unsigned long g_echo_end_time = 0;
static volatile bool g_echo_received = false;

// Machine à états pour gérer le cycle ping/écho
enum class SensorPingState { IDLE, WAITING_FOR_ECHO };
static SensorPingState g_sensor_state = SensorPingState::IDLE;
static unsigned long g_ping_sent_time = 0;

// --- Constantes ---
const unsigned long PING_INTERVAL_MS = 60; // Temps minimum entre les pings
const unsigned long PING_TIMEOUT_MS = 30;  // Temps max d'attente d'un écho (30ms ~ 5m de distance)

// --- Routine de Service d'Interruption ---
static void echo_isr() {
  if (digitalRead(ECHO) == HIGH) {
    g_echo_start_time = micros();
  } else {
    g_echo_end_time = micros();
    g_echo_received = true;
  }
}

// --- Fonctions Publiques ---
inline void sensor_init() {
  pinMode(TRIGGER, OUTPUT);
  pinMode(ECHO, INPUT);
  attachInterrupt(digitalPinToInterrupt(ECHO), echo_isr, CHANGE);
}

inline void sensor_update_task(Robot& robot) {
  unsigned long current_time = millis();

  switch (g_sensor_state) {
    case SensorPingState::IDLE:
      // S'il est temps d'envoyer un nouveau ping
      if (current_time - g_ping_sent_time > PING_INTERVAL_MS) {
        g_ping_sent_time = current_time;
        g_echo_received = false; // Réinitialiser le drapeau avant l'envoi

        // Envoyer l'impulsion de déclenchement de 10µs
        digitalWrite(TRIGGER, LOW);
        delayMicroseconds(2);
        digitalWrite(TRIGGER, HIGH);
        delayMicroseconds(10);
        digitalWrite(TRIGGER, LOW);

        g_sensor_state = SensorPingState::WAITING_FOR_ECHO; // Passer à l'état d'attente
      }
      break;

    case SensorPingState::WAITING_FOR_ECHO:
      // Cas 1: Nous avons reçu un écho valide
      if (g_echo_received) {
        noInterrupts();
        unsigned long duration = g_echo_end_time - g_echo_start_time;
        interrupts();
        // Le "nombre magique" 58 est une approximation pour (1 / (vitesse_du_son_cm_par_us / 2))
        robot.dusm = duration / 58;
        g_sensor_state = SensorPingState::IDLE; // Réinitialiser pour le prochain ping
      } 
      // Cas 2: L'écho n'est pas revenu à temps (timeout)
      else if (current_time - g_ping_sent_time > PING_TIMEOUT_MS) {
        robot.dusm = -1; // Utiliser -1 pour indiquer une lecture invalide ou hors de portée
        g_sensor_state = SensorPingState::IDLE; // Réinitialiser pour le prochain ping
      }
      break;
  }
}

#endif // SENSOR_TASK_H