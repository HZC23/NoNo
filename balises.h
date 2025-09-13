#ifndef BALISES_H
#define BALISES_H

// Removed unused LED timer variables - now using non-blocking approach


void balise_jaune() {
#if ENABLE_LEDS
  // LED should be configured in setup() - no need for pinMode here
  // Use non-blocking approach with millis() for timing
  static unsigned long lastToggle = 0;
  static bool ledState = false;
  static int blinkCount = 0;
  
  unsigned long currentTime = millis();
  
  if (currentTime - lastToggle >= LED_BLINK_INTERVAL) {
    ledState = !ledState;
    digitalWrite(LED_JAUNE, ledState ? HIGH : LOW);
    lastToggle = currentTime;
    
    if (!ledState) { // Count when LED turns off
      blinkCount++;
      if (blinkCount >= LED_BLINK_COUNT) {
        digitalWrite(LED_JAUNE, LOW);
        blinkCount = 0; // Reset for next call
        return;
      }
    }
  }
#else
  if (DEBUG_MODE) Serial.println("LED jaune desactivee (hardware non installe)");
#endif
}

void balise_rouge() {
#if ENABLE_LEDS
  // LED should be configured in setup() - no need for pinMode here
  // Use non-blocking approach with millis() for timing
  static unsigned long lastToggle = 0;
  static bool ledState = false;
  static int blinkCount = 0;
  
  unsigned long currentTime = millis();
  
  if (currentTime - lastToggle >= LED_BLINK_INTERVAL) {
    ledState = !ledState;
    digitalWrite(LED_ROUGE, ledState ? HIGH : LOW);
    lastToggle = currentTime;
    
    if (!ledState) { // Count when LED turns off
      blinkCount++;
      if (blinkCount >= LED_BLINK_COUNT) {
        digitalWrite(LED_ROUGE, LOW);
        blinkCount = 0; // Reset for next call
        return;
      }
    }
  }
#else
  if (DEBUG_MODE) Serial.println("LED rouge desactivee (hardware non installe)");
#endif
}

// === FONCTIONS DE RAPPORT D'ERREURS VISUEL ===

// Fonction pour afficher un code d'erreur avec des clignotements
void showErrorCode(int errorCode) {
#if ENABLE_LEDS
  if (DEBUG_MODE) {
    Serial.print("Code d'erreur visuel: ");
    Serial.println(errorCode);
  }
  
  // Patterns de clignotement pour chaque code d'erreur
  int patterns[][10] = {
    {},
    {200, 200},                   // 1 - Compass init (1 clignotement)
    {200, 200, 200, 200},         // 2 - Compass read (2 clignotements)
    {200, 200, 200, 200, 200, 200}, // 3 - Battery low (3 clignotements)
    {200, 200, 200, 200, 200, 200, 200, 200}, // 4 - Ultrasonic (4 clignotements)
    {200, 200, 200, 200, 200, 200, 200, 200, 200, 200} // 5 - Motor stuck (5 clignotements)
  };
  
  int patternLengths[] = {0, 2, 4, 6, 8, 10};
  
  if (errorCode >= 1 && errorCode <= 5) {
    blinkErrorPattern(patterns[errorCode], patternLengths[errorCode]);
  }
#else
  if (DEBUG_MODE) {
    Serial.print("Code d'erreur (LEDs desactivees): ");
    Serial.println(errorCode);
  }
#endif
}

// Fonction pour exécuter un pattern de clignotement non-bloquant
void blinkErrorPattern(int pattern[], int patternLength) {
#if ENABLE_LEDS
  static unsigned long lastBlinkTime = 0;
  static int currentBlink = 0;
  static bool ledState = false;
  static bool patternActive = false;
  
  unsigned long currentTime = millis();
  
  // Démarrer un nouveau pattern si pas déjà actif
  if (!patternActive) {
    patternActive = true;
    currentBlink = 0;
    ledState = false;
    lastBlinkTime = currentTime;
    if (DEBUG_MODE) Serial.println("Debut du pattern d'erreur");
  }
  
  // Exécuter le pattern
  if (currentBlink < patternLength) {
    if (currentTime - lastBlinkTime >= pattern[currentBlink]) {
      ledState = !ledState;
      digitalWrite(LED_ROUGE, ledState ? HIGH : LOW);
      lastBlinkTime = currentTime;
      
      if (!ledState) { // Quand la LED s'éteint, passer au prochain intervalle
        currentBlink++;
      }
    }
  } else {
    // Pattern terminé
    digitalWrite(LED_ROUGE, LOW);
    patternActive = false;
    if (DEBUG_MODE) Serial.println("Fin du pattern d'erreur");
  }
#endif
}

// Removed old blocking LED functions - replaced with non-blocking versions above

#endif // BALISES_H