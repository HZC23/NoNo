#include "terminal.h"

// Use the global robot object from NoNo.ino
extern Robot robot; 
// Use the global preferences object from NoNo.ino
extern Preferences preferences; 


/**
 * @brief Parses and executes a command string.
 * This function can be called from any source (Serial, BLE, etc.).
 * @param command The command string to parse.
 */
void processCommand(String command) {
    if (command.length() == 0) return;

    if (DEBUG_MODE) {
        Serial.print(F("Command received: ["));
        Serial.print(command);
        Serial.println(F("]"));
    }

    // It's safer to work with a char array buffer
    char cmdBuffer[CMD_BUFFER_SIZE];
    command.toCharArray(cmdBuffer, sizeof(cmdBuffer));

    char* strtok_state;
    char* token = strtok_r(cmdBuffer, CMD_DELIMITER, &strtok_state);

    if (token == nullptr) return;

    // --- MOVEMENT COMMANDS (e.g., "CMD:MOVE=100,50") ---
    if (strcasecmp(token, "CMD") == 0) {
        char* sub_token = strtok_r(NULL, "=", &strtok_state);
        char* value_str = strtok_r(NULL, "", &strtok_state); // Get the rest of the string
        
        if (sub_token != nullptr && value_str != nullptr) {
            if (strcasecmp(sub_token, "MOVE") == 0) {
                int velocity = 0;
                int turn = 0;
                sscanf(value_str, "%d,%d", &velocity, &turn);

                int pwmLeft = velocity + turn;
                int pwmRight = velocity - turn;

                pwmLeft = constrain(pwmLeft, -PWM_MAX, PWM_MAX);
                pwmRight = constrain(pwmRight, -PWM_MAX, PWM_MAX);

                motorA.motorGo(pwmRight);
                motorB.motorGo(pwmLeft);

                if (velocity == 0 && turn == 0) {
                    changeState(robot, IDLE);
                } else {
                    changeState(robot, GAMEPAD_CONTROL);
                }
                robot.lastAppCommandTime = millis();
            }
            // --- GOTO HEADING ---
            else if (strcasecmp(sub_token, "GOTO") == 0) {
                robot.capCibleRotation = atof(value_str);
                changeState(robot, FOLLOW_HEADING);
            }
            // --- SET SPEED ---
            else if (strcasecmp(sub_token, "SPEED") == 0) {
                robot.vitesseCible = constrain(atoi(value_str), 0, PWM_MAX);
            }
            // --- SET COMPASS OFFSET ---
            else if (strcasecmp(sub_token, "COMPASS_OFFSET") == 0) {
                robot.compassOffset = atof(value_str);
            }
            // --- SET COMMUNICATION MODE ---
            else if (strcasecmp(sub_token, "SET_MODE") == 0) {
                int new_mode = -1;
                if (strcasecmp(value_str, "XBOX") == 0) {
                    new_mode = COMM_MODE_XBOX;
                } else { // Any other value will implicitly set Serial mode
                    new_mode = COMM_MODE_SERIAL;
                }

                if (new_mode != -1 && new_mode != robot.activeCommMode) {
                    preferences.begin(NVS_NAMESPACE, false); // Open NVS in read-write mode
                    preferences.putInt(NVS_COMM_MODE_KEY, new_mode);
                    preferences.end();

                    String msg = "Switching to " + String(value_str) + " mode. Rebooting...";
                    Serial.println(msg);

                    delay(1000); // Wait for messages to be sent
                    ESP.restart();
                } else if (new_mode == robot.activeCommMode) {
                    Serial.println("Already in requested mode.");
                } else {
                    Serial.println("Unknown mode requested.");
                }
            }
        }
    }
    // --- MODE COMMANDS (e.g., "MODE:AVOID") ---
    else if (strcasecmp(token, "MODE") == 0) {
        char* value = strtok_r(NULL, "", &strtok_state);
        if (value != nullptr) {
            if (strcasecmp(value, "AVOID") == 0) changeState(robot, OBSTACLE_AVOIDANCE);
            else if (strcasecmp(value, "SENTRY") == 0) changeState(robot, SENTRY_MODE);
            else if (strcasecmp(value, "CALIBRATE") == 0) changeState(robot, CALIBRATING_COMPASS);
            else if (strcasecmp(value, "IDLE") == 0) changeState(robot, IDLE);
            else if (strcasecmp(value, "TOGGLE_AVOID") == 0) {
                if (robot.currentState == OBSTACLE_AVOIDANCE) changeState(robot, IDLE);
                else changeState(robot, OBSTACLE_AVOIDANCE);
            }
            else if (strcasecmp(value, "TOGGLE_SENTRY") == 0) {
                if (robot.currentState == SENTRY_MODE) changeState(robot, IDLE);
                else changeState(robot, SENTRY_MODE);
            }
        }
    }
    // --- HEADLIGHT (e.g., "HL:ON") ---
    else if (strcasecmp(token, "HL") == 0) {
        char* value = strtok_r(NULL, "", &strtok_state);
        if (value != nullptr) {
            if (strcasecmp(value, "ON") == 0) digitalWrite(PIN_PHARE, HIGH);
            else if (strcasecmp(value, "OFF") == 0) digitalWrite(PIN_PHARE, LOW);
            else if (strcasecmp(value, "TOGGLE") == 0) digitalWrite(PIN_PHARE, !digitalRead(PIN_PHARE));
        }
    }
}