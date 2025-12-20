#ifndef TERMINAL_H
#define TERMINAL_H

#include <Arduino.h>
#include <ctype.h>
#include "config.h"
#include "state.h"
#include "fonctions_motrices.h"
#include "compass.h"
#include "support.h" // For Phare control



extern TaskHandle_t musicTaskHandle; // Declare musicTaskHandle as extern

void musicTask(void *pvParameters); // Forward declaration



// Custom implementation of strcasecmp for Arduino compatibility
inline int strcasecmp_local(const char *s1, const char *s2) {
    while (*s1 && (tolower((unsigned char)*s1) == tolower((unsigned char)*s2))) {
        s1++;
        s2++;
    }
    return tolower((unsigned char)*s1) - tolower((unsigned char)*s2);
}

// --- Defines for the command parser ---
#define CMD_DELIMITER ":"
#define CMD_HEADLIGHT "HL"
#define CMD_MODE "M"
#define CMD_SPEED "S"
#define CMD_MUSIC "MUSIC"
#define CMD_TOGGLE_AVOIDANCE "AVOID"
#define CMD_TOGGLE_SENTRY "SENTRY"
#define CMD_START_3D_SCAN "SCAN3D"

/**
 * @brief Parses and executes commands from the Bluetooth Serial port in a non-blocking, memory-safe way.
 *
 * This function reads incoming Bluetooth serial data byte-by-byte into a static buffer,
 * avoiding the use of the Arduino String class to prevent heap fragmentation.
 * A command is processed when a newline character ('\n') is received.
 * It expects commands in the format "V:100;D:45\n".
 *
 * @param robot The main robot state object.
 */
inline void Terminal(Robot& robot) {
    //  BODY COMMENTED OUT TO REMOVE BluetoothSerial dependency
    /*
    static char cmdBuffer[CMD_BUFFER_SIZE];
    static uint8_t cmdIndex = 0;

    while (SerialBT.available() > 0) { // Read from Bluetooth Serial
        char incomingChar = SerialBT.read();

        if (incomingChar == '\n') {
            cmdBuffer[cmdIndex] = '\0';

            if (cmdIndex > 0) {
                if (DEBUG_MODE) {
                    Serial.print(F("App Command received: ["));
                    Serial.print(cmdBuffer);
                    Serial.println(F("]"));
                }

                char* strtok_state;
                char* command = strtok_r(cmdBuffer, CMD_DELIMITER, &strtok_state);
                char* value = strtok_r(NULL, CMD_DELIMITER, &strtok_state);

                // Movement commands (V:100;D:45 format)
                if (command[0] == 'V' && command[1] == ':') {
                    int velocity = atoi(&command[2]);
                    int direction = atoi(value); // If a direction is provided directly after V:
                    
                    // If the original command was just "V:100" without "D:45", then value would be NULL or not a number.
                    // We need to re-parse from the original buffer if the structure is V:X;D:Y
                    char* semi = strchr(cmdBuffer, ';');
                    if (semi) {
                        char* d_command = semi + 1;
                        if (d_command[0] == 'D' && d_command[1] == ':') {
                            direction = atoi(&d_command[2]);
                        }
                    } else {
                        // If no semicolon, then it means V:100, no explicit D given, so assume straight
                        direction = 0;
                    }


                    int scaledVelocity = map(velocity, -100, 100, -PWM_MAX, PWM_MAX);
                    int scaledDirection = map(direction, -100, 100, -PWM_MAX, PWM_MAX);

                    int pwmLeft = scaledVelocity + scaledDirection;
                    int pwmRight = scaledVelocity - scaledDirection;

                    pwmLeft = constrain(pwmLeft, -PWM_MAX, PWM_MAX);
                    pwmRight = constrain(pwmRight, -PWM_MAX, PWM_MAX);

                    motorA.motorGo(pwmRight); // Assuming motorA is right
                    motorB.motorGo(pwmLeft);  // Assuming motorB is left

                    if (scaledVelocity == 0 && scaledDirection == 0) {
                        changeState(robot, IDLE);
                    } else {
                        changeState(robot, APP_CONTROL); 
                    }
                }
                // Headlight command
                else if (strcasecmp_local(command, CMD_HEADLIGHT) == 0) {
                    if (strcasecmp_local(value, "ON") == 0) {
                        digitalWrite(PIN_PHARE, HIGH);
                    } else if (strcasecmp_local(value, "OFF") == 0) {
                        digitalWrite(PIN_PHARE, LOW);
                    }
                }
                // Mode commands
                else if (strcasecmp_local(command, CMD_MODE) == 0) {
                    if (strcasecmp_local(value, "AVOID") == 0) {
                        changeState(robot, OBSTACLE_AVOIDANCE);
                    } else if (strcasecmp_local(value, "SENTRY") == 0) {
                        changeState(robot, SENTRY_MODE);
                    } else if (strcasecmp_local(value, "SCAN3D") == 0) {
                        changeState(robot, SCANNING_3D);
                    } else if (strcasecmp_local(value, "IDLE") == 0) {
                        changeState(robot, IDLE);
                    }
                }
                // Toggle Obstacle Avoidance
                else if (strcasecmp_local(command, CMD_TOGGLE_AVOIDANCE) == 0) {
                    if (robot.currentState == OBSTACLE_AVOIDANCE) {
                        changeState(robot, IDLE);
                    } else {
                        changeState(robot, OBSTACLE_AVOIDANCE);
                    }
                }
                // Toggle Sentry Mode
                else if (strcasecmp_local(command, CMD_TOGGLE_SENTRY) == 0) {
                    if (robot.currentState == SENTRY_MODE) {
                        changeState(robot, IDLE);
                    } else {
                        changeState(robot, SENTRY_MODE);
                    }
                }
                // Start 3D Scan
                else if (strcasecmp_local(command, CMD_START_3D_SCAN) == 0) {
                    changeState(robot, SCANNING_3D);
                }
                // Speed command
                else if (strcasecmp_local(command, CMD_SPEED) == 0) {
                    int speed = atoi(value);
                    robot.vitesseCible = constrain(speed, 0, PWM_MAX);
                }
                // Music command
                else if (strcasecmp_local(command, CMD_MUSIC) == 0) {
                    if (strcasecmp_local(value, "PLAY") == 0) {
                        if (musicTaskHandle == NULL) {
                            xTaskCreatePinnedToCore(
                                musicTask,         // Task function
                                "MusicTask",       // Name of the task
                                4096,              // Stack size
                                NULL,              // Parameter
                                1,                 // Priority
                                &musicTaskHandle,  // Task handle
                                0                  // Core ID
                            );
                        }
                    } else if (strcasecmp_local(value, "STOP") == 0) {
                        if (musicTaskHandle != NULL) {
                            vTaskDelete(musicTaskHandle);
                            musicTaskHandle = NULL;
                            noTone(BUZZER_PIN); // Stop any ongoing tone
                        }
                    }
                }
                
                robot.lastAppCommandTime = millis(); // Update last command time
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
    */
}


#endif // TERMINAL_H
