#include "comms.h"
#include "robot.h"
#include "logger.h"
#include "hardware.h" // For hardware objects used by xbox controller and others
#include "battery_utils.h"
#include <ArduinoJson.h>
#include <Preferences.h>
#include <cstring>  // For strcasecmp, strncpy, strcmp, strchr

// --- Global Xbox Controller Instance ---
XboxControllerBluepad xboxController(robot);

// --- Xbox Controller Static Member Definitions ---
Robot* XboxControllerBluepad::robot_ptr = nullptr;
ControllerPtr XboxControllerBluepad::myControllers[BP32_MAX_CONTROLLERS];

// Struct to hold the previous state of a controller for single-press detection
struct ControllerState {
    uint8_t dpad = 0;
    uint16_t miscButtons = 0;
    uint32_t buttons = 0;
};
static ControllerState lastStates[BP32_MAX_CONTROLLERS];


// --- Haptics Implementation ---
void trigger_rumble(uint8_t duration, uint8_t weak_magnitude, uint8_t strong_magnitude) {
    for (int i = 0; i < BP32_MAX_CONTROLLERS; i++) {
        ControllerPtr ctl = XboxControllerBluepad::getController(i);
        if (ctl && ctl->isConnected()) {
            ctl->playDualRumble(0, duration, weak_magnitude, strong_magnitude);
        }
    }
}

// --- Terminal Implementation ---
void checkSerial() {
    static char cmdBuffer[CMD_BUFFER_SIZE];
    static uint8_t cmdIndex = 0;
    while (Serial.available() > 0) {
        char incomingChar = Serial.read();
        if (incomingChar == '\n' || incomingChar == '\r') {
            if (cmdIndex > 0) {
                cmdBuffer[cmdIndex] = '\0';
                processCommand(cmdBuffer);  // Pass char array directly, no String
                cmdIndex = 0;
            }
        } else if (cmdIndex < CMD_BUFFER_SIZE - 1) {
            cmdBuffer[cmdIndex++] = incomingChar;
        }
    }
}

void processCommand(const char* command) {
    // Safety checks - use standard C functions only (no String class)
    if (command == nullptr || command[0] == '\0') return;
    LOG_DEBUG("Command received: [%s]", command);
    
    // Copy to local buffer for parsing (strtok_r modifies its input)
    char cmdBuffer[CMD_BUFFER_SIZE];
    strncpy(cmdBuffer, command, sizeof(cmdBuffer) - 1);
    cmdBuffer[sizeof(cmdBuffer) - 1] = '\0';
    
    // Parse command format: "TOKEN:value_str" (e.g., "M:100,50" or "E:AVOID")
    char* strtok_state;
    char* token = strtok_r(cmdBuffer, ":", &strtok_state);
    if (token == nullptr) return;
    
    char* value_str = strtok_r(NULL, "", &strtok_state);
    int value = (value_str != nullptr) ? atoi(value_str) : 0;

    if (strcasecmp(token, "M") == 0) {
        // Manual movement command: M:velocity,turn
        if (value_str != nullptr) {
            int velocity = 0, turn = 0;
            sscanf(value_str, "%d,%d", &velocity, &turn);
            robot.manualTargetVelocity = velocity;
            robot.manualTargetTurn = turn;
            if (velocity == 0 && turn == 0) { 
                changeState(robot, IDLE); 
            } else { 
                changeState(robot, MANUAL_COMMAND_MODE); 
            }
        }
    } 
    else if (strcasecmp(token, "E") == 0) {
        // State change command: E:AVOID|SENTRY|IDLE
        if (value_str != nullptr) {
            if (strcasecmp(value_str, "AVOID") == 0) {
                changeState(robot, SMART_AVOIDANCE);
            } else if (strcasecmp(value_str, "SENTRY") == 0) {
                changeState(robot, SENTRY_MODE);
            } else if (strcasecmp(value_str, "IDLE") == 0) {
                changeState(robot, IDLE);
            }
        }
    } 
    else if (strcasecmp(token, "lcd_log") == 0) {
        // LCD logging toggle: lcd_log:0|1
        robot.lcdLogsEnabled = (value == 1);
        LOG_INFO("LCD logs set to: %d", robot.lcdLogsEnabled);
        if (!robot.lcdLogsEnabled && lcdAvailable) {
            lcd->clear();
            lcd->setCursor(0, 0);
            lcd->print("LCD Logs OFF");
        }
    } 
    else if (strcasecmp(token, "sm") == 0) {
        // Communication mode setting: sm:xbox|serial
        if (value_str != nullptr) {
            Preferences preferences;
            CommunicationMode newMode = robot.activeCommMode;
            if (strcasecmp(value_str, "xbox") == 0) {
                newMode = COMM_MODE_XBOX;
            } else if (strcasecmp(value_str, "serial") == 0) {
                newMode = COMM_MODE_SERIAL;
            }

            if (newMode != robot.activeCommMode) {
                robot.activeCommMode = newMode;
                preferences.begin(NVS_NAMESPACE, false);
                preferences.putInt(NVS_COMM_MODE_KEY, newMode);
                preferences.end();
                LOG_INFO("Comm mode set to %s. Reboot required to apply.", value_str);
            } else {
                LOG_INFO("Comm mode is already %s.", value_str);
            }
        }
    }
    // Additional commands can be added here with the same pattern
}

// --- Telemetry & State ---
void sendTelemetry(Robot& robot) {
    JsonDocument doc;
    doc["state"] = stateToString(robot.currentState);
    doc["heading"] = robot.cap;
    doc["distance"] = robot.dusm;
    doc["distanceLaser"] = robot.distanceLaser;
    doc["battery"] = readBatteryPercentage();
    doc["speedTarget"] = robot.targetSpeed;
    serializeJson(doc, Serial);
    Serial.println();
}

// --- Display Implementation ---
void setLcdText(Robot& robot, const char* text, bool isContinuation) {
    // Safety check: if LCD is not available, just log and return
    if (!lcdAvailable) {
        LOG_DEBUG("setLcdText called but LCD not available: %s", text);
        return;
    }

    if (!isContinuation && strcmp(text, robot.lcdText) == 0) {
        return;
    }
    
    strncpy(robot.lcdText, text, sizeof(robot.lcdText) - 1);
    robot.lcdText[sizeof(robot.lcdText) - 1] = '\0';

    robot.currentPage = 0;
    robot.customMessageSetTime = millis();
    robot.lastLcdPageTime = millis(); // Reset page timer

    int len = strlen(robot.lcdText);
    if (len > 32) { // 2 lines of 16 chars
        robot.lcdAnimationState = ANIM_SCROLLING_MESSAGE;
        robot.lcdMessageTotalPages = (len + 31) / 32;
    } else {
        robot.lcdAnimationState = ANIM_IDLE;
    }

    // Display first page
    lcd->clear();
    char pageBuffer[17]; // 16 chars + null
    strncpy(pageBuffer, robot.lcdText, 16);
    pageBuffer[16] = '\0';
    lcd->setCursor(0, 0);
    lcd->print(pageBuffer);

    if (len > 16) {
        strncpy(pageBuffer, robot.lcdText + 16, 16);
        pageBuffer[16] = '\0';
        lcd->setCursor(0, 1);
        lcd->print(pageBuffer);
    }
}

void handleLcdAnimations(Robot& robot) {
    if (robot.lcdAnimationState != ANIM_SCROLLING_MESSAGE) {
        return;
    }

    if (millis() - robot.lastLcdPageTime > SCROLL_DELAY_MS) {
        robot.currentPage++;
        if (robot.currentPage >= robot.lcdMessageTotalPages) {
            robot.currentPage = 0; // Loop back
        }

        int offset = robot.currentPage * 32;
        size_t textLen = strlen(robot.lcdText);
        
        // Skip if offset is beyond the text length
        if (offset >= textLen) {
            robot.currentPage = 0;
            offset = 0;
            textLen = strlen(robot.lcdText);
        }
        
        lcd->clear();
        char pageBuffer[17]; // 16 chars + null

        // Line 1
        size_t remaining = textLen - offset;
        size_t line1Len = (remaining > 16) ? 16 : remaining;
        strncpy(pageBuffer, robot.lcdText + offset, line1Len);
        pageBuffer[line1Len] = '\0';
        lcd->setCursor(0, 0);
        lcd->print(pageBuffer);

        // Line 2
        if (remaining > 16) {
            remaining -= 16;
            size_t line2Len = (remaining > 16) ? 16 : remaining;
            strncpy(pageBuffer, robot.lcdText + offset + 16, line2Len);
            pageBuffer[line2Len] = '\0';
            lcd->setCursor(0, 1);
            lcd->print(pageBuffer);
        }
        
        robot.lastLcdPageTime = millis();
    }
}


void displayRandomJoke(Robot& robot) {
    char jokeBuffer[MAX_LCD_TEXT_LENGTH + 1];
    getRandomJokeFromSD(robot, "/jokes.txt", jokeBuffer, sizeof(jokeBuffer));
    setLcdText(robot, jokeBuffer, false);
    robot.lastJokeDisplayTime = millis();
}

void displayJokesIfIdle(Robot& robot) {
    if (robot.currentState != IDLE) return;
    if (robot.lcdAnimationState != ANIM_IDLE || (millis() - robot.customMessageSetTime < CUSTOM_MESSAGE_DURATION_MS)) {
        return;
    }
    unsigned long currentTime = millis();
    if (currentTime - robot.lastLcdUpdateTime >= LCD_IDLE_TIMEOUT_MS) {
        if (currentTime - robot.lastJokeDisplayTime >= LCD_JOKE_INTERVAL_MS || robot.lastJokeDisplayTime == 0) {
            displayRandomJoke(robot);
        }
    }
}

void updateLcdDisplay(Robot& robot) {
    // This function provides a default information screen.
    // It should not run if another message or animation is active.
    if (!robot.lcdLogsEnabled) return;
    if (robot.lcdAnimationState != ANIM_IDLE || 
        (millis() - robot.customMessageSetTime < CUSTOM_MESSAGE_DURATION_MS)) {
        return;
    }

    static unsigned long lastLcdInfoTime = 0;
    if (millis() - lastLcdInfoTime < 250) { // Update interval: 250ms
        return;
    }
    lastLcdInfoTime = millis();
    robot.lastLcdUpdateTime = millis(); // Reset the idle joke timer

    char line1[17];
    char line2[17];
    
    snprintf(line1, sizeof(line1), "E:%-8.8s B:%3d", stateToString(robot.currentState), readBatteryPercentage());

    if (robot.lcdInfoMode == LCD_INFO_SIMPLE) {
        snprintf(line2, sizeof(line2), "V: %3d H: %3d", robot.targetSpeed, (int)robot.cap);
    } else { // LCD_INFO_SENSORS
        if (robot.currentState == MANUAL_COMMAND_MODE) {
            snprintf(line2, sizeof(line2), "V%3d U%3d L%3d", robot.targetSpeed, robot.dusm > 999 ? 999 : robot.dusm, robot.distanceLaser > 999 ? 999 : robot.distanceLaser);
        } else {
            snprintf(line2, sizeof(line2), "H%3d U%3d L%3d", (int)robot.cap, robot.dusm > 999 ? 999 : robot.dusm, robot.distanceLaser > 999 ? 999 : robot.distanceLaser);
        }
    }

    static char lastLine1[17] = "";
    static char lastLine2[17] = "";
    if (strcmp(line1, lastLine1) != 0 || strcmp(line2, lastLine2) != 0) {
        lcd->clear(); // Clear only when content changes
        lcd->setCursor(0, 0);
        lcd->print(line1);
        strcpy(lastLine1, line1);
        
        lcd->setCursor(0, 1);
        lcd->print(line2);
        strcpy(lastLine2, line2);
    }
}


// --- Xbox Controller Class ---
ControllerPtr XboxControllerBluepad::getController(int index) {
    if (index < 0 || index >= BP32_MAX_CONTROLLERS) return nullptr;
    return myControllers[index];
}

XboxControllerBluepad::XboxControllerBluepad(Robot& robotRef) : robot(robotRef) {
    robot_ptr = &robotRef;
}

void XboxControllerBluepad::onConnectedController(ControllerPtr ctl) {
    for (int i = 0; i < BP32_MAX_CONTROLLERS; i++) {
        if (myControllers[i] == nullptr) {
            LOG_INFO("CALLBACK: Controller connected, index=%d", i);
            myControllers[i] = ctl;
            lastStates[i] = ControllerState();
            trigger_rumble(50, 0, 150);
            break;
        }
    }
}

void XboxControllerBluepad::onDisconnectedController(ControllerPtr ctl) {
    for (int i = 0; i < BP32_MAX_CONTROLLERS; i++) {
        if (myControllers[i] == ctl) {
            LOG_INFO("CALLBACK: Controller disconnected, index=%d", i);
            myControllers[i] = nullptr;
            if (robot_ptr->currentState == MANUAL_COMMAND_MODE) {
                changeState(*robot_ptr, IDLE);
            }
            break;
        }
    }
}

void XboxControllerBluepad::begin() {
    LOG_INFO("Initializing Bluepad32...");
    BP32.setup(&XboxControllerBluepad::onConnectedController, &XboxControllerBluepad::onDisconnectedController);
}

bool XboxControllerBluepad::isConnected() {
    for (int i = 0; i < BP32_MAX_CONTROLLERS; i++) {
        if (myControllers[i] && myControllers[i]->isConnected()) return true;
    }
    return false;
}

void XboxControllerBluepad::processControllers() {
    BP32.update();
    

    
    for (int i = 0; i < BP32_MAX_CONTROLLERS; i++) {
        ControllerPtr ctl = myControllers[i];
        if (ctl && ctl->isConnected() && ctl->hasData()) {
            uint32_t currentButtons = ctl->buttons();
            uint8_t currentDpad = ctl->dpad();
            uint16_t currentMiscButtons = ctl->miscButtons();
            ControllerState& last = lastStates[i];

            #define WAS_MAIN_BUTTON_PRESSED(button) ((currentButtons & button) && !(last.buttons & button))
            #define WAS_DPAD_PRESSED(button) ((currentDpad & button) && !(last.dpad & button))
            #define WAS_MISC_BUTTON_PRESSED(button) ((currentMiscButtons & button) && !(last.miscButtons & button))

            if (WAS_MAIN_BUTTON_PRESSED(XBOX_BTN_TOGGLE_MANUAL)) {
                LOG_INFO("XBOX: TOGGLE_MANUAL button pressed");
                trigger_rumble(40, 120, 0);
                if (robot_ptr->currentState == MANUAL_COMMAND_MODE) {
                    changeState(*robot_ptr, IDLE);
                } else {
                    if (robot_ptr->targetSpeed == 0) robot_ptr->targetSpeed = VITESSE_MOYENNE;
                    changeState(*robot_ptr, MANUAL_COMMAND_MODE);
                }
            }
            if (WAS_MAIN_BUTTON_PRESSED(XBOX_BTN_TOGGLE_AVOIDANCE)) {
                LOG_INFO("XBOX: TOGGLE_AVOIDANCE button pressed");
                trigger_rumble(40, 120, 0);
                if (robot_ptr->currentState == SMART_AVOIDANCE) changeState(*robot_ptr, IDLE);
                else changeState(*robot_ptr, SMART_AVOIDANCE);
            }
            if (WAS_MAIN_BUTTON_PRESSED(XBOX_BTN_TOGGLE_HEADLIGHT)) {
                LOG_INFO("XBOX: TOGGLE_HEADLIGHT (Button A) pressed - headlight was %s", robot_ptr->headlightOn ? "ON" : "OFF");
                trigger_rumble(40, 120, 0);
                robot_ptr->headlightOn = !robot_ptr->headlightOn;
                if (robot_ptr->headlightOn) {
                    LOG_INFO("Headlight ON via Xbox");
                    headlightOn();
                } else {
                    LOG_INFO("Headlight OFF via Xbox");
                    headlightOff();
                }
            }
            if (WAS_MAIN_BUTTON_PRESSED(XBOX_BTN_SPEED_UP)) {
                trigger_rumble(25, 80, 0);
                robot_ptr->targetSpeed = constrain(robot_ptr->targetSpeed + XBOX_SPEED_INCREMENT, 0, PWM_MAX);
                LOG_DEBUG("targetSpeed changed to: %d", robot_ptr->targetSpeed);
            }
            if (WAS_MAIN_BUTTON_PRESSED(XBOX_BTN_SPEED_DOWN)) {
                trigger_rumble(25, 80, 0);
                robot_ptr->targetSpeed = constrain(robot_ptr->targetSpeed - XBOX_SPEED_INCREMENT, 0, PWM_MAX);
                LOG_DEBUG("targetSpeed changed to: %d", robot_ptr->targetSpeed);
            }

            if (WAS_MISC_BUTTON_PRESSED(XBOX_BTN_TOGGLE_LCD_MODE)) {
                if (robot_ptr->lcdInfoMode == LCD_INFO_SIMPLE) {
                    robot_ptr->lcdInfoMode = LCD_INFO_SENSORS;
                } else {
                    robot_ptr->lcdInfoMode = LCD_INFO_SIMPLE;
                }
            }

            if (WAS_DPAD_PRESSED(DPAD_RIGHT)) {
                displayRandomJoke(*robot_ptr);
            }

            if (WAS_DPAD_PRESSED(DPAD_UP)) {
                if (XBOX_DPAD_UP_ACTION == XBOX_ACTION_CALIBRATE_COMPASS) {
                    LOG_INFO("D-Pad Up pressed: Initiating compass calibration.");
                    changeState(*robot_ptr, CALIBRATING_COMPASS);
                }
            }

            if (WAS_DPAD_PRESSED(DPAD_DOWN)) {
                if (XBOX_DPAD_DOWN_ACTION == XBOX_ACTION_TOGGLE_MSC) {
                    toggle_msc_mode();
                }
            }

            if (robot_ptr->currentState == MANUAL_COMMAND_MODE) {
                // Read all axes and triggers
                int32_t joyY_left = ctl->axisY();
                int32_t joyX_left = ctl->axisX();
                int32_t lt_val = ctl->brake();
                int32_t rt_val = ctl->throttle();

                // --- Movement Control (Left Stick + Triggers) ---
                if (lt_val > 150 || rt_val > 150) {
                    // Differential Pivot Turn Mode (Triggers)
                    Servodirection.write(robot_ptr->servoNeutralDir);
                    robot_ptr->manualTargetVelocity = 0;
                    robot_ptr->manualTargetTurn = map(rt_val - lt_val, -1023, 1023, robot_ptr->targetSpeed, -robot_ptr->targetSpeed);

                } else {
                    // Ackermann Steering Mode (Left Stick)
                    // Velocity from Left Stick Y
                    robot_ptr->manualTargetVelocity = map(joyY_left, XBOX_JOYSTICK_MIN, XBOX_JOYSTICK_MAX, robot_ptr->targetSpeed, -robot_ptr->targetSpeed);
                    if(abs(joyY_left) < XBOX_JOYSTICK_DEADZONE) robot_ptr->manualTargetVelocity = 0;

                    // Steering angle from Left Stick X
                    int targetSteeringAngle = map(joyX_left, XBOX_JOYSTICK_MIN, XBOX_JOYSTICK_MAX, robot_ptr->servoDirMax, robot_ptr->servoDirMin);
                    targetSteeringAngle = constrain(targetSteeringAngle, robot_ptr->servoDirMin, robot_ptr->servoDirMax);
                    if(abs(joyX_left) < XBOX_JOYSTICK_DEADZONE) {
                      Servodirection.write(robot_ptr->servoNeutralDir); // Center steering if joystick is near center
                      robot_ptr->manualTargetSteeringAngle = robot_ptr->servoNeutralDir;
                    } else {
                      Servodirection.write(targetSteeringAngle);
                      robot_ptr->manualTargetSteeringAngle = targetSteeringAngle;
                    }
                    
                    // In Ackermann mode, manualTargetTurn is not directly used for turning motors,
                    // but we might need to set it to 0 or use it for speed differential later.
                    robot_ptr->manualTargetTurn = 0; 
                }

            } else {
                // If not in manual mode, ensure targets are zero
                robot_ptr->manualTargetVelocity = 0;
                robot_ptr->manualTargetTurn = 0;
            }
            last.buttons = currentButtons;
            last.dpad = currentDpad;
            last.miscButtons = currentMiscButtons;
        }
    }
}

// --- New function for toggling MSC ---
void toggle_msc_mode() {
    LOG_INFO("Toggle MSC mode requested via controller.");
    robot.requestMscToggle = true;
}
