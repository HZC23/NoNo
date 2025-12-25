#include "comms.h"
#include "hardware.h" // For hardware objects used by xbox controller and others
#include "logger.h" // Include the new logger
#include <ArduinoJson.h>

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
                processCommand(String(cmdBuffer));
                cmdIndex = 0;
            }
        } else if (cmdIndex < CMD_BUFFER_SIZE - 1) {
            cmdBuffer[cmdIndex++] = incomingChar;
        }
    }
}

void processCommand(String command) {
    if (command.length() == 0) return;
    LOG_DEBUG("Command received: [%s]", command.c_str());
    char cmdBuffer[CMD_BUFFER_SIZE];
    command.toCharArray(cmdBuffer, sizeof(cmdBuffer));
    char* strtok_state;
    char* token = strtok_r(cmdBuffer, ":", &strtok_state);
    if (token == nullptr) return;
    char* value_str = strtok_r(NULL, "", &strtok_state);

    if (strcasecmp(token, "M") == 0) {
        if (value_str != nullptr) {
            int velocity = 0; int turn = 0;
            sscanf(value_str, "%d,%d", &velocity, &turn);
            robot.manualTargetVelocity = velocity;
            robot.manualTargetTurn = turn;
            if (velocity == 0 && turn == 0) { changeState(robot, IDLE); }
            else { changeState(robot, MANUAL_COMMAND_MODE); }
        }
    } else if (strcasecmp(token, "E") == 0) {
        if (value_str != nullptr) {
            if (strcasecmp(value_str, "AVOID") == 0) changeState(robot, SMART_AVOIDANCE);
            else if (strcasecmp(value_str, "SENTRY") == 0) changeState(robot, SENTRY_MODE);
            else if (strcasecmp(value_str, "IDLE") == 0) changeState(robot, IDLE);
        }
    }
    // ... other commands can be added here
}

// --- Telemetry & State ---
const char* stateToString(RobotState state) {
    switch (state) {
        case IDLE: return "IDLE";
        case MOVING_FORWARD: return "MOVING_FORWARD";
        case MOVING_BACKWARD: return "MOVING_BACKWARD";
        case TURNING_LEFT: return "TURNING_LEFT";
        case TURNING_RIGHT: return "TURNING_RIGHT";
        case MANUAL_COMMAND_MODE: return "MANUAL_COMMAND_MODE";
        case OBSTACLE_AVOIDANCE: return "OBSTACLE_AVOIDANCE";
        case SMART_AVOIDANCE: return "SMART_AVOIDANCE";
        case EMERGENCY_EVASION: return "EMERGENCY_EVASION";
        case STUCK: return "STUCK";
        case SENTRY_MODE: return "SENTRY_MODE";
        default: return "UNKNOWN";
    }
}

void sendTelemetry(Robot& robot) {
    JsonDocument doc;
    doc["state"] = stateToString(robot.currentState);
    doc["heading"] = robot.cap;
    doc["distance"] = robot.dusm;
    doc["distanceLaser"] = robot.distanceLaser;
    doc["battery"] = readBatteryPercentage();
    doc["speedTarget"] = robot.vitesseCible;
    serializeJson(doc, Serial);
    Serial.println();
}

void sendPeriodicData(Robot& robot) {
  if (millis() - robot.lastReportTime > robot.reportInterval) {
    robot.lastReportTime = millis();
    sendTelemetry(robot);
  }
}

// --- Display Implementation ---
void _applyWordWrap(const char* in, char* out, size_t bufferSize, int lineLength) {
    memset(out, 0, bufferSize);
    const char* source = in;
    int outIdx = 0;
    while (strlen(source) > 0 && outIdx < bufferSize - 2) {
        if ((int)strlen(source) <= lineLength) {
            strcat(out, source);
            break;
        }
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
        if (breakPoint == -1) {
            breakPoint = lineLength;
        }
        strncat(out, source, breakPoint);
        outIdx += breakPoint;
        out[outIdx++] = '\n';
        out[outIdx] = '\0';
        source += breakPoint + (source[breakPoint] == ' ' ? 1 : 0);
    }
}

void setLcdText(Robot& robot, const char* text, bool isContinuation) {
    if (robot.lcdAnimationState == Robot::LcdAnimationState::ANIM_IDLE && strncmp(text, robot.lcdText, MAX_LCD_TEXT_LENGTH) == 0) {
        return;
    }
    if (!isContinuation) {
        robot.lcdAnimationState = Robot::LcdAnimationState::ANIM_IDLE;
        memset(robot.lcdText, 0, MAX_LCD_TEXT_LENGTH + 1);
        memset(robot.lcdFormattedText, 0, sizeof(robot.lcdFormattedText));
        robot.lcdAnimationIndex = 0;
        robot.lcdCursorX = 0;
        robot.lcdCursorY = 0;
        lcd->clear();
        lcd->setCursor(0, 0);
    }
    if (isContinuation && strncmp(text, robot.lcdText, MAX_LCD_TEXT_LENGTH) == 0) {
        return;
    }
    if (millis() - robot.lastJokeDisplayTime < (LCD_JOKE_INTERVAL_MS - 100)) {
        return;
    }
    if (!isContinuation) {
        strncpy(robot.lcdText, text, MAX_LCD_TEXT_LENGTH);
    } else {
        strncat(robot.lcdText, text, MAX_LCD_TEXT_LENGTH - strlen(robot.lcdText) - 1);
    }
    robot.lcdText[MAX_LCD_TEXT_LENGTH] = '\0';
    _applyWordWrap(robot.lcdText, robot.lcdFormattedText, sizeof(robot.lcdFormattedText), LCD_LINE_LENGTH);
    robot.lcdAnimationState = Robot::LcdAnimationState::ANIM_TYPEWRITER;
    robot.lcdAnimationNextCharTime = millis();
}

void handleLcdAnimations(Robot& robot) {
    unsigned long currentTime = millis();
    if (robot.lcdAnimationState == Robot::LcdAnimationState::ANIM_IDLE) {
        return;
    }
    // ... (rest of the complex animation logic from Nono_full.ino)
}

void displayRandomJoke(Robot& robot) {
    char jokeBuffer[MAX_LCD_TEXT_LENGTH + 1];
    getRandomJokeFromSD(robot, "/jokes.txt", jokeBuffer, sizeof(jokeBuffer));
    setLcdText(robot, jokeBuffer);
    robot.lastJokeDisplayTime = millis();
}

void displayJokesIfIdle(Robot& robot) {
    if (robot.currentState != IDLE) return;
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

void updateLcdDisplay(Robot& robot) {
    if (robot.lcdAnimationState != Robot::LcdAnimationState::ANIM_IDLE || 
        (millis() - robot.lastJokeDisplayTime < (LCD_JOKE_INTERVAL_MS)) ||
        (millis() - robot.customMessageSetTime < CUSTOM_MESSAGE_DURATION_MS)) {
        return;
    }
    char displayBuffer[MAX_LCD_TEXT_LENGTH + 1] = {0};
    // Using the same logic from a previous step to have named states
    snprintf(displayBuffer, sizeof(displayBuffer), "Etat: %s", stateToString(robot.currentState));
    setLcdText(robot, displayBuffer);
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
            ControllerState& last = lastStates[i];

            #define WAS_MAIN_BUTTON_PRESSED(button) ((currentButtons & button) && !(last.buttons & button))

            if (WAS_MAIN_BUTTON_PRESSED(XBOX_BTN_TOGGLE_MANUAL)) {
                trigger_rumble(40, 120, 0);
                if (robot_ptr->currentState == MANUAL_COMMAND_MODE) {
                    changeState(*robot_ptr, IDLE);
                } else {
                    if (robot_ptr->vitesseCible == 0) robot_ptr->vitesseCible = VITESSE_MOYENNE;
                    changeState(*robot_ptr, MANUAL_COMMAND_MODE);
                }
            }
            if (WAS_MAIN_BUTTON_PRESSED(XBOX_BTN_TOGGLE_AVOIDANCE)) {
                trigger_rumble(40, 120, 0);
                if (robot_ptr->currentState == SMART_AVOIDANCE) changeState(*robot_ptr, IDLE);
                else changeState(*robot_ptr, SMART_AVOIDANCE);
            }
            if (WAS_MAIN_BUTTON_PRESSED(XBOX_BTN_SPEED_UP)) {
                trigger_rumble(25, 80, 0);
                robot_ptr->vitesseCible = constrain(robot_ptr->vitesseCible + XBOX_SPEED_INCREMENT, 0, PWM_MAX);
            }
            if (WAS_MAIN_BUTTON_PRESSED(XBOX_BTN_SPEED_DOWN)) {
                trigger_rumble(25, 80, 0);
                robot_ptr->vitesseCible = constrain(robot_ptr->vitesseCible - XBOX_SPEED_INCREMENT, 0, PWM_MAX);
            }

            if (robot_ptr->currentState == MANUAL_COMMAND_MODE) {
                int32_t joyY_left = ctl->axisY();
                int32_t joyY_right = ctl->axisRY();
                if (abs(joyY_left) < 150) joyY_left = 0;
                if (abs(joyY_right) < 150) joyY_right = 0;
                int max_speed = robot_ptr->vitesseCible;
                int left_motor_speed = map(joyY_left, -1024, 1023, max_speed, -max_speed);
                int right_motor_speed = map(joyY_right, -1024, 1023, max_speed, -max_speed);
                robot_ptr->manualTargetVelocity = (left_motor_speed + right_motor_speed) / 2;
                robot_ptr->manualTargetTurn = (right_motor_speed - left_motor_speed) / 2; // Corrected turn calculation
            } else {
                robot_ptr->manualTargetVelocity = 0;
                robot_ptr->manualTargetTurn = 0;
            }
            last.buttons = currentButtons;
        }
    }
}