
#ifndef DISPLAY_H
#define DISPLAY_H

#include <Arduino.h>
#include "state.h"

inline void setLcdText(Robot& robot, const String& text) {
    if (text == robot.lcdText) return; // Avoid unnecessary screen redraws
    lcd.clear();

    if (text.length() <= 16) {
        lcd.setCursor(0, 0);
        lcd.print(text);
        lcd.setCursor(0, 1);
        lcd.print("                "); // Clear second line
    } else { // text.length() > 16 and <= 32 (checked in terminal.h)
        lcd.setCursor(0, 0);
        lcd.print(text.substring(0, 16));
        lcd.setCursor(0, 1);
        lcd.print(text.substring(16));
    }
    robot.lcdText = text;
}

// Wrapper for backward compatibility if needed, though direct use is better
inline void setLcdText(const String& text) {
    lcd.clear();
    lcd.print(text);
}

#endif // DISPLAY_H
