#ifndef FONCTIONS_MOTRICES_H
#define FONCTIONS_MOTRICES_H

#define SERVO_TURN_OFFSET 20
#define PIR_TURN_SPEED 50

// === FONCTIONS DE BASE ===

void changeState(RobotState newState) {
  // Validate state transition
  if (newState < 0 || newState > SMART_TURNING) {
    if (DEBUG_MODE) {
      Serial.print("ERREUR: État invalide: ");
      Serial.println(newState);
    }
    return;
  }
  
  RobotState previousState = currentState;
  currentState = newState;
  actionStarted = false;
  lastActionTime = millis();
  
  // Réinitialiser les variables d'évitement si on sort du mode AVOID_MANEUVER
  if (newState != AVOID_MANEUVER) {
    hasReculed = false;
    hasTurned = false;
  }
  
  // Log state transition
  if (DEBUG_MODE) {
    Serial.print("Transition d'état: ");
    Serial.print(previousState);
    Serial.print(" -> ");
    Serial.println(newState);
  }
}

// Add other motor control functions here

#endif // FONCTIONS_MOTRICES_H