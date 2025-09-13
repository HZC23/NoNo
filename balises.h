
#ifndef BALISES_H
#define BALISES_H

#include "config.h"

// Prototypes
void balise_jaune();
void balise_rouge();
void balises_off();

// Implementations
inline void balise_jaune() {
  #if ENABLE_LEDS
    digitalWrite(LED_ROUGE, LOW);
    digitalWrite(LED_JAUNE, HIGH);
  #endif
}

inline void balise_rouge() {
  #if ENABLE_LEDS
    digitalWrite(LED_JAUNE, LOW);
    digitalWrite(LED_ROUGE, HIGH);
  #endif
}

inline void balises_off() {
  #if ENABLE_LEDS
    digitalWrite(LED_ROUGE, LOW);
    digitalWrite(LED_JAUNE, LOW);
  #endif
}

#endif // BALISES_H
