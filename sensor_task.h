#ifndef SENSOR_TASK_H
#define SENSOR_TASK_H

#include <Arduino.h>
#include "config.h"
#include "state.h"

// --- Fonctions Publiques ---

// Initialise les broches du capteur à ultrasons.
inline void sensor_init() {
  pinMode(TRIGGER, OUTPUT);
  pinMode(ECHO, INPUT);
}

// Met à jour la mesure du capteur à ultrasons de manière non-bloquante (sans interruptions).
// Cette fonction doit être appelée régulièrement dans la boucle principale.
inline void sensor_update_task(Robot& robot) {
  static unsigned long last_ping_time = 0;
  unsigned long current_time = millis();

  // Envoyer un ping seulement si l'intervalle minimum est écoulé.
  if (current_time - last_ping_time > ULTRASONIC_PING_INTERVAL_MS) {
    last_ping_time = current_time;

    // Envoyer l'impulsion de déclenchement de 10µs pour démarrer une mesure.
    digitalWrite(TRIGGER, LOW);
    delayMicroseconds(ULTRASONIC_TRIGGER_PULSE_LOW_US);
    digitalWrite(TRIGGER, HIGH);
    delayMicroseconds(ULTRASONIC_TRIGGER_PULSE_HIGH_US);
    digitalWrite(TRIGGER, LOW);

    // Mesurer la durée de l'écho avec pulseIn().
    // pulseIn() est bloquant, mais avec un timeout raisonnable, l'impact est limité.
    // C'est une alternative robuste lorsque les interruptions ne sont pas fiables ou disponibles.
    unsigned long duration = pulseIn(ECHO, HIGH, ULTRASONIC_PULSE_TIMEOUT_US);

    if (duration > 0) {
      // La formule standard pour convertir la durée (en microsecondes) en distance (en cm).
      // duration / 2 / 29.1 (vitesse du son) => duration / 58.2
      robot.dusm = duration / ULTRASONIC_DURATION_TO_CM_DIVISOR;
    } else {
      // Si pulseIn() retourne 0, cela signifie qu'aucun écho n'a été reçu avant le timeout.
      robot.dusm = ULTRASONIC_ERROR_VALUE; // Utiliser une valeur négative pour indiquer une erreur ou hors de portée.
    }
  }
}

#endif // SENSOR_TASK_H