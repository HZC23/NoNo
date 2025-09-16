#ifndef TERMINAL_H
#define TERMINAL_H

#include <Arduino.h>
#include <strings.h> // For strcasecmp
#include "state.h"
#include "fonctions_motrices.h"
#include "compass.h"
#include "support.h" // For Phare control

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
    // Static buffer to hold incoming command characters.
    // Static variables retain their value between function calls.
    static char cmdBuffer[CMD_BUFFER_SIZE];
    static uint8_t cmdIndex = 0;

    while (Serial.available() > 0) {
        char incomingChar = Serial.read();

        // Process the command if a newline is received.
        if (incomingChar == '\n') {
            cmdBuffer[cmdIndex] = '\0'; // Null-terminate the string

            if (cmdIndex > 0) { // Ensure we have a command to process
                if (DEBUG_MODE) {
                    Serial.print(F("Command received: ["));
                    Serial.print(cmdBuffer);
                    Serial.println(F("]"));
                }

                // --- Begin Parsing ---
                char* strtok_state; // For re-entrant strtok_r
                char* type = strtok_r(cmdBuffer, CMD_DELIMITER, &strtok_state);

                if (type != NULL && strcasecmp(type, "CMD") == 0) {
                    char* action = strtok_r(NULL, CMD_DELIMITER, &strtok_state);
                    char* value = strtok_r(NULL, CMD_DELIMITER, &strtok_state);

                    if (action != NULL && value != NULL) {
                        // --- Process Actions ---
                        if (strcasecmp(action, "MOVE") == 0) {
                            if (strcasecmp(value, "FWD") == 0) {
                                robot.vitesseCible = VITESSE_MOYENNE;
                                changeState(robot, MOVING_FORWARD);
                            } else if (strcasecmp(value, "BWD") == 0) {
                                robot.vitesseCible = VITESSE_LENTE;
                                changeState(robot, MOVING_BACKWARD);
                            } else if (strcasecmp(value, "LEFT") == 0) {
                                changeState(robot, MANUAL_TURNING_LEFT);
                            } else if (strcasecmp(value, "RIGHT") == 0) {
                                changeState(robot, MANUAL_TURNING_RIGHT);
                            } else if (strcasecmp(value, "STOP") == 0) {
                                robot.vitesseCible = 0;
                                changeState(robot, IDLE);
                            }
                        } else if (strcasecmp(action, "SPEED") == 0) {
                            robot.vitesseCible = constrain(atoi(value), 0, 255);
                            if (DEBUG_MODE) {
                                Serial.print(F("Vitesse cible mise a: "));
                                Serial.println(robot.vitesseCible);
                            }
                        } else if (strcasecmp(action, "GOTO") == 0) {
                            robot.Ncap = atoi(value);
                            robot.vitesseCible = VITESSE_MOYENNE;
                            changeState(robot, FOLLOW_HEADING);
                        } else if (strcasecmp(action, "TURN") == 0) {
                            robot.capCibleRotation = atof(value);
                            changeState(robot, TURNING_LEFT);
                        } else if (strcasecmp(action, "LIGHT") == 0) {
                            if (strcasecmp(value, "ON") == 0) {
                                PhareAllume();
                            } else if (strcasecmp(value, "OFF") == 0) {
                                PhareEteint();
                            }
                        } else if (strcasecmp(action, "CALIBRATE") == 0) {
                            if (strcasecmp(value, "COMPASS") == 0) {
                                changeState(robot, CALIBRATING_COMPASS);
                            }
                        }
                    } else {
                         if (DEBUG_MODE) Serial.println(F("Error: Missing action or value"));
                    }
                }
            }

            // Reset buffer for the next command
            cmdIndex = 0;

        } else if (cmdIndex < CMD_BUFFER_SIZE - 1) {
            // Add character to buffer if it's not a carriage return and there's space
            if (incomingChar != '\r') {
                cmdBuffer[cmdIndex++] = incomingChar;
            }
        } else {
            // Buffer overflowed. Discard the command and reset.
            if (DEBUG_MODE) {
                Serial.println(F("Error: Command buffer overflow"));
            }
            cmdIndex = 0;
        }
    }
}

#endif // TERMINAL_H