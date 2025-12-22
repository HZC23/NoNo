#ifndef DISPLAY_H
#define DISPLAY_H

#include <Arduino.h>
#include "state.h"
#include "config.h"
#include "DFRobot_RGBLCD1602.h"
#include "sd_utils.h"
#include <string.h>

// Declare extern if lcd object is defined in NoNo.ino
extern DFRobot_RGBLCD1602 *lcd;

/**
 * @brief Applies word wrap to a string.
 */
inline void _applyWordWrap(const char* in, char* out, size_t bufferSize, int lineLength) {
    memset(out, 0, bufferSize);
    
    const char* source = in;
    int outIdx = 0;

    while (strlen(source) > 0 && outIdx < bufferSize - 2) {
        // If the remaining text fits on one line, just copy it and we're done.
        if ((int)strlen(source) <= lineLength) {
            strcat(out, source);
            break;
        }

        // Find the last space within the line length limit.
        int breakPoint = -1;
        for (int i = 0; i <= lineLength; i++) {
            if (source[i] == ' ') {
                breakPoint = i;
            }
            if (source[i] == '\0') {
                breakPoint = -1; 
                break;
            }
        }
        
        // If no space was found (a very long word), we must break the word.
        if (breakPoint == -1) {
            breakPoint = lineLength;
        }

        // Copy the part of the string up to the breakpoint.
        strncat(out, source, breakPoint);
        outIdx += breakPoint;
        
        // Add a newline.
        out[outIdx++] = '\n';
        out[outIdx] = '\0'; // Ensure null termination for strcat

        // Advance the source pointer. Skip the space at the breakpoint if it was a space.
        source += breakPoint + (source[breakPoint] == ' ' ? 1 : 0);
    }
}


/**
 * @brief Sets the target text for the LCD and starts the animation.
 */
inline void setLcdText(Robot& robot, const char* text) {
    // 1. If text is identical and animation is idle, do nothing.
    if (robot.lcdAnimationState == Robot::LcdAnimationState::ANIM_IDLE && strncmp(text, robot.lcdText, MAX_LCD_TEXT_LENGTH) == 0) {
        return;
    }
    
    // Prevent changing text while a joke is being displayed
    if (millis() - robot.lastJokeDisplayTime < (LCD_JOKE_INTERVAL_MS - 100)) {
        return;
    }

    // 2. The text has changed. Copy the new raw text.
    strncpy(robot.lcdText, text, MAX_LCD_TEXT_LENGTH);
    robot.lcdText[MAX_LCD_TEXT_LENGTH] = '\0'; // Ensure null termination

    // 3. Apply word wrap to the raw text.
    _applyWordWrap(robot.lcdText, robot.lcdFormattedText, sizeof(robot.lcdFormattedText), LCD_LINE_LENGTH);
    if (DEBUG_MODE) {
        Serial.println("--- LCD DEBUG: setLcdText ---");
        Serial.print("Raw In: '"); Serial.print(robot.lcdText); Serial.println("'");
        Serial.print("Wrapped: '"); Serial.print(robot.lcdFormattedText); Serial.println("'");
    }

    // 4. Reset and start the animation state.
    robot.lcdAnimationState = Robot::LcdAnimationState::ANIM_TYPEWRITER;
    robot.lcdAnimationIndex = 0;
    robot.lcdAnimationNextCharTime = millis();
    robot.lcdCursorX = 0;
    robot.lcdCursorY = 0;

    lcd->clear();
    lcd->setCursor(0, 0);
}


/**
 * @brief The core animation engine for the LCD. MUST be called in the main loop.
 */
inline void handleLcdAnimations(Robot& robot) {
    unsigned long currentTime = millis();

    if (robot.lcdAnimationState == Robot::LcdAnimationState::ANIM_IDLE) {
        return;
    }

    if (DEBUG_MODE) {
        Serial.print("LCD_DEBUG: State="); Serial.print(robot.lcdAnimationState);
        Serial.print(" | Index="); Serial.print(robot.lcdAnimationIndex);
    }

    switch(robot.lcdAnimationState) {
        case Robot::LcdAnimationState::ANIM_TYPEWRITER: {
            if (currentTime < robot.lcdAnimationNextCharTime) {
                if (DEBUG_MODE) Serial.println(" | Waiting for next char time.");
                return;
            }

            char nextChar = robot.lcdFormattedText[robot.lcdAnimationIndex];
            if (DEBUG_MODE) { Serial.print(" | Char='"); Serial.print(nextChar); Serial.println("'"); }

            if (nextChar == '\0') {
                // End of all text.
                robot.lcdAnimationState = Robot::LcdAnimationState::ANIM_IDLE;
                robot.lastLcdUpdateTime = currentTime;
                if (DEBUG_MODE) Serial.println("LCD_DEBUG: Animation finished, moving to IDLE.");
                return;
            }

            if (nextChar == '\n') {
                robot.lcdCursorY++;
                robot.lcdCursorX = 0;

                // If we've filled the screen, pause for scrolling.
                if (robot.lcdCursorY >= LCD_ROWS) {
                    if (robot.lcdFormattedText[robot.lcdAnimationIndex + 1] != '\0') {
                        robot.lcdAnimationState = Robot::LcdAnimationState::ANIM_SCROLL_PAUSE;
                        robot.lastLcdUpdateTime = currentTime; // This is the trigger for the pause delay.
                        if (DEBUG_MODE) Serial.println("LCD_DEBUG: End of page, moving to SCROLL_PAUSE.");
                    } else {
                        robot.lcdAnimationState = Robot::LcdAnimationState::ANIM_IDLE;
                        robot.lastLcdUpdateTime = currentTime;
                        if (DEBUG_MODE) Serial.println("LCD_DEBUG: Animation finished (end of page), moving to IDLE.");
                    }
                    return;
                }
                lcd->setCursor(robot.lcdCursorX, robot.lcdCursorY);

            } else {
                // Print the character.
                lcd->print(nextChar);
                robot.lcdCursorX++;
            }

            robot.lcdAnimationIndex++;
            robot.lcdAnimationNextCharTime = currentTime + LCD_TYPEWRITER_DELAY_MS;
            break;
        }

        case Robot::LcdAnimationState::ANIM_SCROLL_PAUSE: {
            if (DEBUG_MODE) { Serial.print(" | Time since update: "); Serial.println(currentTime - robot.lastLcdUpdateTime); }
            if (currentTime - robot.lastLcdUpdateTime >= SCROLL_DELAY_MS) {
                if (DEBUG_MODE) Serial.println("LCD_DEBUG: Scroll delay finished. Executing scroll.");
                // --- Perform the scroll animation ---
                int line_start_index = -1;
                int newline_count = 0;
                for (int i = 0; i < robot.lcdAnimationIndex; ++i) {
                    if (robot.lcdFormattedText[i] == '\n') {
                        newline_count++;
                        if (newline_count == robot.lcdCursorY - 1) {
                            line_start_index = i + 1;
                            break;
                        }
                    }
                }
                
                if (line_start_index != -1) {
                    char prev_line[LCD_LINE_LENGTH + 1] = {0};
                    int j = 0;
                    for (int i = line_start_index; robot.lcdFormattedText[i] != '\n' && robot.lcdFormattedText[i] != '\0' && j < LCD_LINE_LENGTH; ++i, ++j) {
                        prev_line[j] = robot.lcdFormattedText[i];
                    }
                    lcd->clear();
                    lcd->setCursor(0, 0);
                    lcd->print(prev_line);
                    robot.lcdCursorY = 1;
                    robot.lcdCursorX = 0;
                    lcd->setCursor(robot.lcdCursorX, robot.lcdCursorY);
                }
                
                robot.lcdAnimationState = Robot::LcdAnimationState::ANIM_TYPEWRITER;
                robot.lcdAnimationIndex++; 
                robot.lcdAnimationNextCharTime = currentTime;
                if (DEBUG_MODE) Serial.println("LCD_DEBUG: Scroll complete, moving back to TYPEWRITER.");
            }
            break;
        }
    }
}


// --- Functions to set display content ---

inline void displayRandomJoke(Robot& robot) {
    char jokeBuffer[MAX_LCD_TEXT_LENGTH + 1];
    getRandomJokeFromSD(robot, "/jokes.txt", jokeBuffer, sizeof(jokeBuffer));
    
    // Set the joke text, which will trigger the animation
    strncpy(robot.lcdText, jokeBuffer, MAX_LCD_TEXT_LENGTH);
    robot.lcdText[MAX_LCD_TEXT_LENGTH] = '\0';
    _applyWordWrap(robot.lcdText, robot.lcdFormattedText, sizeof(robot.lcdFormattedText), LCD_LINE_LENGTH);
    
    robot.lcdAnimationState = Robot::LcdAnimationState::ANIM_TYPEWRITER;
    robot.lcdAnimationIndex = 0;
    robot.lcdAnimationNextCharTime = millis();
    robot.lcdCursorX = 0;
    robot.lcdCursorY = 0;
    lcd->clear();
    lcd->setCursor(0, 0);

    robot.lastJokeDisplayTime = millis();
}

inline void displayJokesIfIdle(Robot& robot) {
    if (robot.lcdAnimationState != Robot::LcdAnimationState::ANIM_IDLE || (millis() - robot.customMessageSetTime < CUSTOM_MESSAGE_DURATION_MS)) {
        return;
    }

    unsigned long currentTime = millis();
    if (currentTime - robot.lastLcdUpdateTime >= LCD_IDLE_TIMEOUT_MS) {
        if (currentTime - robot.lastJokeDisplayTime >= LCD_JOKE_INTERVAL_MS || robot.lastJokeDisplayTime == 0) {
            displayRandomJoke(robot);
        }
    }
}


inline void updateLcdDisplay(Robot& robot) {
    // DO NOT update status text if any animation is running, a joke was just shown, or a custom message is active.
    if (robot.lcdAnimationState != Robot::LcdAnimationState::ANIM_IDLE || 
        (millis() - robot.lastJokeDisplayTime < (LCD_JOKE_INTERVAL_MS)) ||
        (millis() - robot.customMessageSetTime < CUSTOM_MESSAGE_DURATION_MS)) {
        return;
    }

    // --- Select text based on state ---
    char displayBuffer[MAX_LCD_TEXT_LENGTH + 1] = {0};
    
    switch (robot.currentState) {
        case IDLE:
            snprintf(displayBuffer, sizeof(displayBuffer), "Etat: IDLE. En attente de vos ordres.");
            break;
        case MOVING_FORWARD:
        case MANUAL_FORWARD:
            snprintf(displayBuffer, sizeof(displayBuffer), "J'avance vers le cap %d", robot.cap);
            break;
        case MOVING_BACKWARD:
        case MANUAL_BACKWARD:
            snprintf(displayBuffer, sizeof(displayBuffer), "Je recule, attention derriere !");
            break;
        case TURNING_LEFT:
        case MANUAL_TURNING_LEFT:
            snprintf(displayBuffer, sizeof(displayBuffer), "Je tourne a gauche vers %d", robot.cap);
            break;
        case TURNING_RIGHT:
        case MANUAL_TURNING_RIGHT:
            snprintf(displayBuffer, sizeof(displayBuffer), "Je tourne a droite vers %d", robot.cap);
            break;
        case OBSTACLE_AVOIDANCE:
        case SMART_AVOIDANCE:
            snprintf(displayBuffer, sizeof(displayBuffer), "Obstacle detecte! J'analyse la situation.");
            break;
        // Add other cases as needed
        default:
            snprintf(displayBuffer, sizeof(displayBuffer), "Etat %d", robot.currentState);
            break;
    }
    
    setLcdText(robot, displayBuffer);
}

#endif // DISPLAY_H