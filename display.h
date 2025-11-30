
#ifndef DISPLAY_H
#define DISPLAY_H

#include <Arduino.h>
#include "state.h"
#include "config.h"
#include "DFRobot_RGBLCD1602.h"
#include "sd_utils.h"
#include <string.h>

// Declare extern if lcd object is defined in NoNo.ino
extern DFRobot_RGBLCD1602 lcd;

inline void setLcdText(Robot& robot, const char* text) {
    if (strcmp(text, robot.lcdText) == 0) return; // Avoid unnecessary screen redraws
    
    strncpy(robot.lcdText, text, MAX_LCD_TEXT_LENGTH);
    robot.lcdText[MAX_LCD_TEXT_LENGTH] = '\0';
    robot.lastLcdUpdateTime = millis();

    lcd.clear();
    
    char line1[LCD_LINE_LENGTH + 1] = {0};
    char line2[LCD_LINE_LENGTH + 1] = {0};

    strncpy(line1, text, LCD_LINE_LENGTH);
    
    if (strlen(text) > LCD_LINE_LENGTH) {
        strncpy(line2, text + LCD_LINE_LENGTH, LCD_LINE_LENGTH);
    }

    lcd.setCursor(0, 0);
    lcd.print(line1);
    lcd.setCursor(0, 1);
    lcd.print(line2);
}

inline void displayRandomJoke(Robot& robot) {
    char jokeBuffer[MAX_LCD_TEXT_LENGTH + 1];
    getRandomJokeFromSD("jokes.txt", jokeBuffer, sizeof(jokeBuffer));
    setLcdText(robot, jokeBuffer);
    robot.lastJokeDisplayTime = millis();
}

void displayJokesIfIdle(Robot& robot) {
    unsigned long currentTime = millis();
    if (currentTime - robot.lastLcdUpdateTime >= LCD_IDLE_TIMEOUT_MS) {
        if (currentTime - robot.lastJokeDisplayTime >= LCD_JOKE_INTERVAL_MS || robot.lastJokeDisplayTime == 0) {
            displayRandomJoke(robot);
        }
    }
}

inline void updateLcdDisplay(Robot& robot) {
    char displayBuffer[MAX_LCD_TEXT_LENGTH + 1] = {0};
    char line1[LCD_LINE_LENGTH + 1] = {0};
    char line2[LCD_LINE_LENGTH + 1] = {0};

    switch (robot.currentState) {
        case IDLE:
            snprintf(displayBuffer, sizeof(displayBuffer), "%-16s%-16s", "IDLE", "Waiting...");
            break;
        case MOVING_FORWARD:
        case MANUAL_FORWARD:
            snprintf(line1, sizeof(line1), "FORWARD");
            snprintf(line2, sizeof(line2), "CAP:%-11d", robot.cap);
            snprintf(displayBuffer, sizeof(displayBuffer), "%-16s%-16s", line1, line2);
            break;
        case MOVING_BACKWARD:
        case MANUAL_BACKWARD:
            snprintf(line1, sizeof(line1), "BACKWARD");
            snprintf(line2, sizeof(line2), "CAP:%-11d", robot.cap);
            snprintf(displayBuffer, sizeof(displayBuffer), "%-16s%-16s", line1, line2);
            break;
        case TURNING_LEFT:
        case MANUAL_TURNING_LEFT:
            snprintf(line1, sizeof(line1), "TURN LEFT");
            snprintf(line2, sizeof(line2), "CAP:%-11d", robot.cap);
            snprintf(displayBuffer, sizeof(displayBuffer), "%-16s%-16s", line1, line2);
            break;
        case TURNING_RIGHT:
        case MANUAL_TURNING_RIGHT:
            snprintf(line1, sizeof(line1), "TURN RIGHT");
            snprintf(line2, sizeof(line2), "CAP:%-11d", robot.cap);
            snprintf(displayBuffer, sizeof(displayBuffer), "%-16s%-16s", line1, line2);
            break;
        case OBSTACLE_AVOIDANCE:
        case SMART_AVOIDANCE:
            snprintf(line1, sizeof(line1), "AVOIDING");
            snprintf(line2, sizeof(line2), "US:%-4d L:%-4d", robot.dusm, robot.distanceLaser);
            snprintf(displayBuffer, sizeof(displayBuffer), "%-16s%-16s", line1, line2);
            break;
        case CALIBRATING_COMPASS:
            snprintf(displayBuffer, sizeof(displayBuffer), "%-16s%-16s", "CALIBRATING", "COMPASS...");
            break;
        case SENTRY_MODE:
            snprintf(displayBuffer, sizeof(displayBuffer), "%-16s%-16s", "SENTRY MODE", "Scanning...");
            break;
        case SENTRY_ALARM:
            snprintf(displayBuffer, sizeof(displayBuffer), "%-16s%-16s", "SENTRY ALARM", "INTRUDER!!!");
            break;
        case FOLLOW_HEADING:
            snprintf(line1, sizeof(line1), "FOLLOW HDG");
            snprintf(line2, sizeof(line2), "TRG:%-4d CUR:%-4d", (int)robot.capCibleRotation, robot.cap);
            snprintf(displayBuffer, sizeof(displayBuffer), "%-16s%-16s", line1, line2);
            break;
        case MAINTAIN_HEADING:
            snprintf(line1, sizeof(line1), "MAINTAIN HDG");
            snprintf(line2, sizeof(line2), "TRG:%-4d CUR:%-4d", (int)robot.capCibleRotation, robot.cap);
            snprintf(displayBuffer, sizeof(displayBuffer), "%-16s%-16s", line1, line2);
            break;
        case SCANNING:
            snprintf(line1, sizeof(line1), "SCANNING");
            snprintf(line2, sizeof(line2), "ANG:%-4d DIST:%-4d", robot.currentScanAngleH, robot.distanceLaser);
            snprintf(displayBuffer, sizeof(displayBuffer), "%-16s%-16s", line1, line2);
            break;
        default:
            snprintf(line1, sizeof(line1), "STATE: %-8d", robot.currentState);
            snprintf(line2, sizeof(line2), "UNKNOWN");
            snprintf(displayBuffer, sizeof(displayBuffer), "%-16s%-16s", line1, line2);
            break;
    }
    setLcdText(robot, displayBuffer);
}

#endif // DISPLAY_H
