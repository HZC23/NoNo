
#ifndef DISPLAY_H
#define DISPLAY_H

#include <Arduino.h>
#include "state.h"

inline void setLcdText(Robot& robot, const String& text) {
    if (text == robot.lcdText) return; // Avoid unnecessary screen redraws
    lcd.clear();

    if (text.length() <= LCD_LINE_LENGTH) {
        lcd.setCursor(0, 0);
        lcd.print(text);
        lcd.setCursor(0, 1);
        char spaces[LCD_LINE_LENGTH + 1];
        memset(spaces, ' ', LCD_LINE_LENGTH);
        spaces[LCD_LINE_LENGTH] = '\0';
        lcd.print(spaces);
    } else { // text.length() > LCD_LINE_LENGTH and <= MAX_LCD_TEXT_LENGTH
        lcd.setCursor(0, 0);
        lcd.print(text.substring(0, LCD_LINE_LENGTH));
        lcd.setCursor(0, 1);
        lcd.print(text.substring(LCD_LINE_LENGTH));
    }
    robot.lcdText = text;
}

#endif // DISPLAY_H
