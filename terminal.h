#ifndef TERMINAL_H
#define TERMINAL_H

#include <Arduino.h>
#include <ctype.h>

#include "state.h"
#include "fonctions_motrices.h"
#include "compass.h"
#include "support.h" // For Phare control

// Custom implementation of strcasecmp for Arduino compatibility
inline int strcasecmp_local(const char *s1, const char *s2) {
    while (*s1 && (tolower((unsigned char)*s1) == tolower((unsigned char)*s2))) {
        s1++;
        s2++;
    }
    return tolower((unsigned char)*s1) - tolower((unsigned char)*s2);
}

// --- Defines for the command parser ---
#define CMD_BUFFER_SIZE 64 // Maximum command length (e.g., "CMD:CALIBRATE:COMPASS")
#define CMD_DELIMITER ":"

/**
 * @brief Parses and executes commands from the Serial port in a non-blocking, memory-safe way.
 * 
 * This function reads incoming serial data byte-by-byte into a static buffer,
 * avoiding the use of the Arduino String class to prevent heap fragmentation.
 * A command is processed when a newline character ('\n') is received.
 * It expects commands in the format "TYPE:ACTION:VALUE".
 * 
 * @param robot The main robot state object.
 */
inline void Terminal(Robot& robot) {
    static char cmdBuffer[CMD_BUFFER_SIZE];
    static uint8_t cmdIndex = 0;

    while (Serial.available() > 0) {
        char incomingChar = Serial.read();

        if (incomingChar == '\n') {
            cmdBuffer[cmdIndex] = '\0';

            if (cmdIndex > 0) {
                if (DEBUG_MODE) {
                    Serial.print(F("Command received: ["));
                    Serial.print(cmdBuffer);
                    Serial.println(F("]"));
                }

                char* strtok_state;
                char* type = strtok_r(cmdBuffer, CMD_DELIMITER, &strtok_state);

                if (type != NULL && strcasecmp_local(type, "CMD") == 0) {
                    char* action = strtok_r(NULL, CMD_DELIMITER, &strtok_state);
                    char* value = strtok_r(NULL, CMD_DELIMITER, &strtok_state);

                    if (action != NULL && value != NULL) {
                        if (strcasecmp_local(action, "MOVE") == 0) {
                            // Special handling for heading modes
                            if (robot.currentState == FOLLOW_HEADING || robot.currentState == MAINTAIN_HEADING) {
                                if (strcasecmp_local(value, "FWD") == 0) {
                                    // Toggle between moving and pausing
                                    if (robot.currentState == MAINTAIN_HEADING) {
                                        changeState(robot, FOLLOW_HEADING);
                                    } else {
                                        changeState(robot, MAINTAIN_HEADING);
                                    }
                                } else if (strcasecmp_local(value, "BWD") == 0) {
                                    changeState(robot, MAINTAIN_HEADING); // Pause
                                } else if (strcasecmp_local(value, "LEFT") == 0) {
                                    robot.Ncap = (robot.Ncap - 5 + 360) % 360; // Adjust target heading
                                } else if (strcasecmp_local(value, "RIGHT") == 0) {
                                    robot.Ncap = (robot.Ncap + 5) % 360; // Adjust target heading
                                } else if (strcasecmp_local(value, "STOP") == 0) {
                                    changeState(robot, IDLE); // Exit heading mode
                                }
                            } else {
                                // Standard movement commands
                                if (strcasecmp_local(value, "FWD") == 0) {
                                    robot.vitesseCible = VITESSE_MOYENNE;
                                    changeState(robot, MOVING_FORWARD);
                                } else if (strcasecmp_local(value, "BWD") == 0) {
                                    robot.vitesseCible = VITESSE_LENTE;
                                    changeState(robot, MOVING_BACKWARD);
                                } else if (strcasecmp_local(value, "LEFT") == 0) {
                                    changeState(robot, MANUAL_TURNING_LEFT);
                                } else if (strcasecmp_local(value, "RIGHT") == 0) {
                                    changeState(robot, MANUAL_TURNING_RIGHT);
                                } else if (strcasecmp_local(value, "STOP") == 0) {
                                    robot.vitesseCible = 0;
                                    changeState(robot, IDLE);
                                }
                            }
                        } else if (strcasecmp_local(action, "SPEED") == 0) {
                            robot.vitesseCible = constrain(atoi(value), 0, 255);
                        } else if (strcasecmp_local(action, "GOTO") == 0) {
                            robot.Ncap = atoi(value);
                            robot.vitesseCible = VITESSE_MOYENNE;
                            changeState(robot, FOLLOW_HEADING);
                        } else if (strcasecmp_local(action, "TURN") == 0) {
                            robot.capCibleRotation = atof(value);
                            changeState(robot, TURNING_LEFT);
                        } else if (strcasecmp_local(action, "LIGHT") == 0) {
                            if (strcasecmp_local(value, "ON") == 0) { PhareAllume(); }
                            else if (strcasecmp_local(value, "OFF") == 0) { PhareEteint(); }
                        } else if (strcasecmp_local(action, "CALIBRATE") == 0) {
                            if (strcasecmp_local(value, "COMPASS") == 0) {
                                changeState(robot, CALIBRATING_COMPASS);
                            }
                        } else if (strcasecmp_local(action, "TURRET") == 0) {
                            if (strcasecmp_local(value, "CENTER") == 0) {
                                tourelle.write(SCAN_CENTER_ANGLE, 90);
                            }
                        } else if (strcasecmp_local(action, "SCAN") == 0) {
                            if (strcasecmp_local(value, "START") == 0) {
                                changeState(robot, SCANNING);
                            }
                        } else if (strcasecmp_local(action, "COMPASS_OFFSET") == 0) {
                            robot.compassOffset = atof(value);
                            saveCompassCalibration(robot); // Persist the new offset
                            if (DEBUG_MODE) {
                                Serial.print(F("Compass offset set to: "));
                                Serial.println(robot.compassOffset);
                            }
                        } else if (strcasecmp_local(action, "MODE") == 0) {
                            if (strcasecmp_local(value, "AVOID") == 0) {
                                robot.Ncap = robot.cap; // Set target to current heading
                                robot.vitesseCible = VITESSE_MOYENNE;
                                changeState(robot, SMART_AVOIDANCE);
                            } else if (strcasecmp_local(value, "SENTRY") == 0) {
                                changeState(robot, SENTRY_MODE);
                            } else if (strcasecmp_local(value, "MANUAL") == 0) {
                                changeState(robot, IDLE);
                            }
                        } else if (strcasecmp_local(action, "LCD") == 0) {
                            if (strlen(value) > 32) {
                                if (DEBUG_MODE) Serial.println(F("Error: LCD text too long (max 32 chars)"));
                            } else {
                                setLcdText(robot, String(value));
                            }
                        }
                    } else {
                         if (DEBUG_MODE) Serial.println(F("Error: Missing action or value"));
                    }
                }
            }
            cmdIndex = 0;
        } else if (cmdIndex < CMD_BUFFER_SIZE - 1) {
            if (incomingChar != '\r') {
                cmdBuffer[cmdIndex++] = incomingChar;
            }
        } else {
            if (DEBUG_MODE) {
                Serial.println(F("Error: Command buffer overflow"));
            }
            cmdIndex = 0;
        }
    }
}

#endif // TERMINAL_H
