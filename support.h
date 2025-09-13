#ifndef SUPPORT_H
#define SUPPORT_H

#include <Arduino.h>
#include "config.h"

// Prototypes
void PhareAllume();
void PhareEteint();
void Batterie();
bool isValidNumericInput(const String& input, int minVal, int maxVal);

// Implementations
inline void PhareAllume() {
  // Placeholder
}

inline void PhareEteint() {
  // Placeholder
}

inline void Batterie() {
  // Placeholder
}

inline bool isValidNumericInput(const String& input, int minVal, int maxVal) {
  for (unsigned int i = 0; i < input.length(); i++) {
    if (!isDigit(input.charAt(i))) {
      return false;
    }
  }
  long val = input.toInt();
  return (val >= minVal && val <= maxVal);
}

#endif // SUPPORT_H