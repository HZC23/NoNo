
#ifndef DISPLAY_H
#define DISPLAY_H

#include <Arduino.h>
#include "state.h"

inline void setLcdText(Robot& robot, const String& text) {
    if (text == robot.lcdText) return; // Avoid unnecessary screen redraws
    lcd.clear();
    lcd.print(text);
    robot.lcdText = text;
}

// Wrapper for backward compatibility if needed, though direct use is better
inline void setLcdText(const String& text) {
    lcd.clear();
    lcd.print(text);
}

#endif // DISPLAY_H
