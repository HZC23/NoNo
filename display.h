
#ifndef DISPLAY_H
#define DISPLAY_H

#include <Arduino.h>
#include "state.h" // Need access to Robot struct and RobotState enum
#include "config.h" // For LCD_LINE_LENGTH and other constants
#include "DFRobot_RGBLCD1602.h" // For lcd object

// Declare extern if lcd object is defined in NoNo.ino
extern DFRobot_RGBLCD1602 lcd;

// Jokes array
const char* jokes[] = {
    "Pourquoi le robot est-il allé à l'école? Pour améliorer ses circuits!",
    "Quel est le plat préféré des robots? Les frites de silicium!",
    "Que dit un robot quand il rencontre un autre robot? 'Salut, fil de fer!'",
    "Quel est le sport préféré des robots? La robot-ique!",
    "Comment un robot se marie-t-il? Il trouve la bonne connexion!",
    "Pourquoi les robots n'ont-ils jamais faim? Parce qu'ils sont toujours remplis d'octets!"
};
const int numJokes = sizeof(jokes) / sizeof(jokes[0]);

// Forward declaration
void displayJokesIfIdle(Robot& robot);

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
    robot.lastLcdUpdateTime = millis(); // Update timestamp on actual display change
}

inline void displayRandomJoke(Robot& robot) {
    // Generate a random index for the joke array
    int jokeIndex = random(numJokes);
    setLcdText(robot, jokes[jokeIndex]);
    robot.lastJokeDisplayTime = millis();
}

void displayJokesIfIdle(Robot& robot) {
    unsigned long currentTime = millis();
    if (currentTime - robot.lastLcdUpdateTime >= LCD_IDLE_TIMEOUT_MS) {
        // LCD has been idle, now check if it's time to change the joke
        if (currentTime - robot.lastJokeDisplayTime >= LCD_JOKE_INTERVAL_MS || robot.lastJokeDisplayTime == 0) { // If it's the first time displaying a joke, lastJokeDisplayTime will be 0
            displayRandomJoke(robot);
        }
    }
}

inline void updateLcdDisplay(Robot& robot) {
    String displayString = ""; // This will hold both lines
    char buffer[LCD_LINE_LENGTH + 1]; // Buffer for formatted strings

    switch (robot.currentState) {
        case IDLE:
            displayString += "IDLE            "; // 16 chars
            displayString += "Waiting...      ";
            break;
        case MOVING_FORWARD:
            snprintf(buffer, sizeof(buffer), "FORWARD         ");
            displayString += buffer;
            snprintf(buffer, sizeof(buffer), "CAP:%-11d", robot.cap); // 16 chars
            displayString += buffer;
            break;
        case MOVING_BACKWARD:
            snprintf(buffer, sizeof(buffer), "BACKWARD        ");
            displayString += buffer;
            snprintf(buffer, sizeof(buffer), "CAP:%-11d", robot.cap);
            displayString += buffer;
            break;
        case TURNING_LEFT:
            snprintf(buffer, sizeof(buffer), "TURN LEFT       ");
            displayString += buffer;
            snprintf(buffer, sizeof(buffer), "CAP:%-11d", robot.cap);
            displayString += buffer;
            break;
        case TURNING_RIGHT:
            snprintf(buffer, sizeof(buffer), "TURN RIGHT      ");
            displayString += buffer;
            snprintf(buffer, sizeof(buffer), "CAP:%-11d", robot.cap);
            displayString += buffer;
            break;
        case MANUAL_FORWARD:
            snprintf(buffer, sizeof(buffer), "MANUAL FWD      ");
            displayString += buffer;
            snprintf(buffer, sizeof(buffer), "CAP:%-11d", robot.cap);
            displayString += buffer;
            break;
        case MANUAL_BACKWARD:
            snprintf(buffer, sizeof(buffer), "MANUAL BACK     ");
            displayString += buffer;
            snprintf(buffer, sizeof(buffer), "CAP:%-11d", robot.cap);
            displayString += buffer;
            break;
        case MANUAL_TURNING_LEFT:
            snprintf(buffer, sizeof(buffer), "MANUAL TURN L   ");
            displayString += buffer;
            snprintf(buffer, sizeof(buffer), "CAP:%-11d", robot.cap);
            displayString += buffer;
            break;
        case MANUAL_TURNING_RIGHT:
            snprintf(buffer, sizeof(buffer), "MANUAL TURN R   ");
            displayString += buffer;
            snprintf(buffer, sizeof(buffer), "CAP:%-11d", robot.cap);
            displayString += buffer;
            break;
        case OBSTACLE_AVOIDANCE:
        case SMART_AVOIDANCE:
            snprintf(buffer, sizeof(buffer), "AVOIDING        ");
            displayString += buffer;
            snprintf(buffer, sizeof(buffer), "US:%-4d L:%-4d", robot.dusm, robot.distanceLaser); // 16 chars
            displayString += buffer;
            break;
        case CALIBRATING_COMPASS:
            displayString += "CALIBRATING     ";
            displayString += "COMPASS...      ";
            break;
        case SENTRY_MODE:
            displayString += "SENTRY MODE     ";
            displayString += "Scanning...     ";
            break;
        case SENTRY_ALARM:
            displayString += "SENTRY ALARM    ";
            displayString += "INTRUDER!!!     ";
            break;
        case FOLLOW_HEADING:
            snprintf(buffer, sizeof(buffer), "FOLLOW HDG      ");
            displayString += buffer;
            snprintf(buffer, sizeof(buffer), "TRG:%-4d CUR:%-4d", (int)robot.capCibleRotation, robot.cap);
            displayString += buffer;
            break;
        case MAINTAIN_HEADING:
            snprintf(buffer, sizeof(buffer), "MAINTAIN HDG    ");
            displayString += buffer;
            snprintf(buffer, sizeof(buffer), "TRG:%-4d CUR:%-4d", (int)robot.capCibleRotation, robot.cap);
            displayString += buffer;
            break;
        case SCANNING:
            snprintf(buffer, sizeof(buffer), "SCANNING        ");
            displayString += buffer;
            snprintf(buffer, sizeof(buffer), "ANG:%-4d DIST:%-4d", robot.currentScanAngleH, robot.distanceLaser);
            displayString += buffer;
            break;
        default:
            snprintf(buffer, sizeof(buffer), "STATE: %-8d", robot.currentState);
            displayString += buffer;
            displayString += "UNKNOWN         ";
            break;
    }
    setLcdText(robot, displayString);
}

#endif // DISPLAY_H
