#include "comms.h"
#include "hardware.h" // For hardware objects used by xbox controller and others
#include <ArduinoJson.h>

// --- Global Xbox Controller Instance ---
XboxControllerBluepad xboxController(robot);

// --- Xbox Controller Static Member Definitions ---
Robot* XboxControllerBluepad::robot_ptr = nullptr;
MX1508* XboxControllerBluepad::motorA_ptr = nullptr;
MX1508* XboxControllerBluepad::motorB_ptr = nullptr;
Tourelle* XboxControllerBluepad::tourelle_ptr = nullptr;
ControllerPtr XboxControllerBluepad::myControllers[BP32_MAX_CONTROLLERS];


// --- Terminal Implementation (from terminal.cpp) ---
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

    if (DEBUG_MODE) {
        Serial.print(F("Command received: ["));
        Serial.print(command);
        Serial.println(F("]"));
    }

    char cmdBuffer[CMD_BUFFER_SIZE];
    command.toCharArray(cmdBuffer, sizeof(cmdBuffer));

    char* strtok_state;
    char* token = strtok_r(cmdBuffer, ":", &strtok_state);
    if (token == nullptr) return;

    char* value_str = strtok_r(NULL, "", &strtok_state);

    if (strcasecmp(token, "M") == 0) {
        if (value_str != nullptr) {
            int velocity = 0;
            int turn = 0;
            sscanf(value_str, "%d,%d", &velocity, &turn);
            int pwmLeft = constrain(velocity + turn, -PWM_MAX, PWM_MAX);
            int pwmRight = constrain(velocity - turn, -PWM_MAX, PWM_MAX);
            motorA.motorGo(pwmRight);
            motorB.motorGo(pwmLeft);
            if (velocity == 0 && turn == 0) changeState(robot, IDLE);
            robot.lastAppCommandTime = millis();
        }
    }
    else if (strcasecmp(token, "G") == 0) {
        if (value_str != nullptr) {
            robot.capCibleRotation = atof(value_str);
            changeState(robot, FOLLOW_HEADING);
        }
    }
    else if (strcasecmp(token, "S") == 0) {
        if (value_str != nullptr) robot.vitesseCible = constrain(atoi(value_str), 0, PWM_MAX);
    }
    else if (strcasecmp(token, "CO") == 0) {
        if (value_str != nullptr) robot.compassOffset = atof(value_str);
    }
    else if (strcasecmp(token, "SM") == 0) {
        if (value_str != nullptr) {
            int new_mode = -1;
            if (strcasecmp(value_str, "XBOX") == 0) new_mode = COMM_MODE_XBOX;
            else new_mode = COMM_MODE_SERIAL;

            if (new_mode != -1 && new_mode != robot.activeCommMode) {
                preferences.begin(NVS_NAMESPACE, false);
                preferences.putInt(NVS_COMM_MODE_KEY, new_mode);
                preferences.end();
                Serial.println("Switching mode. Rebooting...");
                delay(1000);
                ESP.restart();
            }
        }
    }
    else if (strcasecmp(token, "E") == 0) {
        if (value_str != nullptr) {
            if (strcasecmp(value_str, "AVOID") == 0) changeState(robot, OBSTACLE_AVOIDANCE);
            else if (strcasecmp(value_str, "SENTRY") == 0) changeState(robot, SENTRY_MODE);
            else if (strcasecmp(value_str, "CALIBRATE") == 0) changeState(robot, CALIBRATING_COMPASS);
            else if (strcasecmp(value_str, "IDLE") == 0) changeState(robot, IDLE);
        }
    }
    else if (strcasecmp(token, "L") == 0) {
        if (value_str != nullptr) {
            if (strcasecmp(value_str, "ON") == 0) digitalWrite(PIN_PHARE, HIGH);
            else if (strcasecmp(value_str, "OFF") == 0) digitalWrite(PIN_PHARE, LOW);
            else if (strcasecmp(value_str, "TOGGLE") == 0) digitalWrite(PIN_PHARE, !digitalRead(PIN_PHARE));
        }
    }
}


// --- Telemetry Implementation (from telemetry.h) ---
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

const char* stateToString(RobotState state) {
    switch (state) {
        case IDLE: return "IDLE";
        case MOVING_FORWARD: return "MOVING_FORWARD";
        case MOVING_BACKWARD: return "MOVING_BACKWARD";
        case TURNING_LEFT: return "TURNING_LEFT";
        case TURNING_RIGHT: return "TURNING_RIGHT";
        case MANUAL_FORWARD: return "MANUAL_FORWARD";
        case MANUAL_BACKWARD: return "MANUAL_BACKWARD";
        case MANUAL_TURNING_LEFT: return "MANUAL_TURNING_LEFT";
        case MANUAL_TURNING_RIGHT: return "MANUAL_TURNING_RIGHT";
        case OBSTACLE_AVOIDANCE: return "OBSTACLE_AVOIDANCE";
        case WAITING_FOR_TURRET: return "WAITING_FOR_TURRET";
        case FOLLOW_HEADING: return "FOLLOW_HEADING";
        case MAINTAIN_HEADING: return "MAINTAIN_HEADING";
        case BACKING_UP_OBSTACLE: return "BACKING_UP_OBSTACLE";
        case SCANNING_FOR_PATH: return "SCANNING_FOR_PATH";
        case TURNING_TO_PATH: return "TURNING_TO_PATH";
        case SMART_TURNING: return "SMART_TURNING";
        case CALIBRATING_COMPASS: return "CALIBRATING_COMPASS";
        case SMART_AVOIDANCE: return "SMART_AVOIDANCE";
        case SENTRY_MODE: return "SENTRY_MODE";
        case CHECKING_GROUND: return "CHECKING_GROUND";
        case CLIFF_DETECTED: return "CLIFF_DETECTED";
        case ANIMATING_HEAD: return "ANIMATING_HEAD";
        case GAMEPAD_CONTROL: return "GAMEPAD_CONTROL";
        case EMERGENCY_EVASION: return "EMERGENCY_EVASION";
        case STUCK: return "STUCK";
        default: return "UNKNOWN";
    }
}


// --- Display Implementation (from display.h) ---
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

    switch(robot.lcdAnimationState) {
        case Robot::LcdAnimationState::ANIM_TYPEWRITER: {
            if (currentTime < robot.lcdAnimationNextCharTime) {
                return;
            }

            char nextChar = robot.lcdFormattedText[robot.lcdAnimationIndex];

            if (nextChar == '\0') {
                robot.lcdAnimationState = Robot::LcdAnimationState::ANIM_IDLE;
                robot.lastLcdUpdateTime = currentTime;
                return;
            }

            if (nextChar == '\n') {
                robot.lcdCursorY++;
                robot.lcdCursorX = 0;

                if (robot.lcdCursorY >= LCD_ROWS) {
                    if (robot.lcdFormattedText[robot.lcdAnimationIndex + 1] != '\0') {
                        robot.lcdAnimationState = Robot::LcdAnimationState::ANIM_SCROLL_PAUSE;
                        robot.lastLcdUpdateTime = currentTime; 
                    } else {
                        robot.lcdAnimationState = Robot::LcdAnimationState::ANIM_IDLE;
                        robot.lastLcdUpdateTime = currentTime;
                    }
                    return;
                }
                
                lcd->setCursor(robot.lcdCursorX, robot.lcdCursorY);

            } else {
                lcd->print(nextChar);
                robot.lcdCursorX++;
                
                if (robot.lcdCursorX >= 16) {
                     robot.lcdCursorX = 0;
                     robot.lcdCursorY++; 
                     lcd->setCursor(robot.lcdCursorX, robot.lcdCursorY);
                }
            }

            robot.lcdAnimationIndex++;
            robot.lcdAnimationNextCharTime = currentTime + LCD_TYPEWRITER_DELAY_MS;
            break;
        }

        case Robot::LcdAnimationState::ANIM_SCROLL_PAUSE: {
            if (currentTime - robot.lastLcdUpdateTime >= SCROLL_DELAY_MS) {
                
                lcd->clear();

                int linesToKeep = LCD_ROWS - 1;
                int scanIndex = robot.lcdAnimationIndex - 1;
                int foundNewlines = 0;
                int startPrintIndex = 0;

                while (scanIndex >= 0) {
                    if (robot.lcdFormattedText[scanIndex] == '\n') {
                        foundNewlines++;
                        if (foundNewlines == linesToKeep) {
                            startPrintIndex = scanIndex + 1;
                            break;
                        }
                    }
                    scanIndex--;
                }

                lcd->setCursor(0, 0);
                int tempCursorY = 0;
                
                for (int i = startPrintIndex; i < robot.lcdAnimationIndex; i++) {
                    char c = robot.lcdFormattedText[i];
                    if (c == '\n') {
                        tempCursorY++;
                        lcd->setCursor(0, tempCursorY);
                    } else {
                        lcd->print(c);
                    }
                }

                robot.lcdCursorX = 0;
                robot.lcdCursorY = LCD_ROWS - 1;
                lcd->setCursor(robot.lcdCursorX, robot.lcdCursorY);

                robot.lcdAnimationState = Robot::LcdAnimationState::ANIM_TYPEWRITER;
                
                robot.lcdAnimationIndex++; 
                robot.lcdAnimationNextCharTime = currentTime + LCD_TYPEWRITER_DELAY_MS;
            }
            break;
        }
    }
}

void displayRandomJoke(Robot& robot) {
    char jokeBuffer[MAX_LCD_TEXT_LENGTH + 1];
    getRandomJokeFromSD(robot, "/jokes.txt", jokeBuffer, sizeof(jokeBuffer));
    
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

void displayJokesIfIdle(Robot& robot) {
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
    
    switch (robot.currentState) {
        case IDLE:
            snprintf(displayBuffer, sizeof(displayBuffer), "Etat: IDLE. En attente de vos ordres.");
            break;
        case MOVING_FORWARD:
            snprintf(displayBuffer, sizeof(displayBuffer), "J'avance vers le cap %d", robot.cap);
            break;
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
        default:
            snprintf(displayBuffer, sizeof(displayBuffer), "Etat %d", robot.currentState);
            break;
    }
    
    setLcdText(robot, displayBuffer);
}


// Struct to hold the previous state of a controller for single-press detection
struct ControllerState {
    uint8_t dpad = 0;
    uint16_t miscButtons = 0;
    uint32_t buttons = 0;
};
static ControllerState lastStates[BP32_MAX_CONTROLLERS];


// --- Constructor & Hardware Setter ---
XboxControllerBluepad::XboxControllerBluepad(Robot& robotRef) : robot(robotRef) {
    robot_ptr = &robotRef;
}

void XboxControllerBluepad::setHardware(MX1508& motorA_ref, MX1508& motorB_ref, Tourelle& tourelle_ref) {
    motorA_ptr = &motorA_ref;
    motorB_ptr = &motorB_ref;
    tourelle_ptr = &tourelle_ref;
}

// --- Bluepad32 Connection Callbacks ---
void XboxControllerBluepad::onConnectedController(ControllerPtr ctl) {
    bool found = false;
    for (int i = 0; i < BP32_MAX_CONTROLLERS; i++) {
        if (myControllers[i] == ctl) {
            found = true;
            break;
        }
    }
    if (!found) {
        for (int i = 0; i < BP32_MAX_CONTROLLERS; i++) {
            if (myControllers[i] == nullptr) {
                Serial.printf("CALLBACK: Controller connected, index=%d\n", i);
                myControllers[i] = ctl;
                lastStates[i] = ControllerState();
                break;
            }
        }
    }
}

void XboxControllerBluepad::onDisconnectedController(ControllerPtr ctl) {
    for (int i = 0; i < BP32_MAX_CONTROLLERS; i++) {
        if (myControllers[i] == ctl) {
            Serial.printf("CALLBACK: Controller disconnected, index=%d\n", i);
            myControllers[i] = nullptr;
            break;
        }
    }
}

// --- Initialization ---
void XboxControllerBluepad::begin() {
    Serial.println("Initializing Bluepad32...");
    BP32.setup(&XboxControllerBluepad::onConnectedController, &XboxControllerBluepad::onDisconnectedController);
}

bool XboxControllerBluepad::isConnected() {
    for (int i = 0; i < BP32_MAX_CONTROLLERS; i++) {
        if (myControllers[i] && myControllers[i]->isConnected()) {
            return true;
        }
    }
    return false;
}


// --- Main Processing Loop ---
void XboxControllerBluepad::processControllers() {
    BP32.update();

    for (int i = 0; i < BP32_MAX_CONTROLLERS; i++) {
        ControllerPtr ctl = myControllers[i];
        if (ctl && ctl->isConnected() && ctl->hasData()) {
            
            int32_t joyX = ctl->axisX();
            int32_t joyY = ctl->axisY();

            if (abs(joyX) < 150) joyX = 0;
            if (abs(joyY) < 150) joyY = 0;

            int throttle = map(joyY, -1024, 1023, PWM_MAX, -PWM_MAX);
            int turn = map(joyX, -1024, 1023, -PWM_MAX, PWM_MAX);
            
            int pwmRight = constrain(throttle - turn, -PWM_MAX, PWM_MAX);
            int pwmLeft = constrain(throttle + turn, -PWM_MAX, PWM_MAX);

            if (robot_ptr->currentState == GAMEPAD_CONTROL) {
                motorA_ptr->motorGo(pwmRight);
                motorB_ptr->motorGo(pwmLeft);
            }

            if (pwmLeft != 0 || pwmRight != 0) {
                if(robot_ptr->currentState != GAMEPAD_CONTROL) changeState(*robot_ptr, GAMEPAD_CONTROL);
            } else if (robot_ptr->currentState == GAMEPAD_CONTROL) {
                changeState(*robot_ptr, IDLE);
            }

            uint32_t currentButtons = ctl->buttons();
            uint16_t currentMiscButtons = ctl->miscButtons();
            ControllerState& last = lastStates[i];

            #define WAS_MAIN_BUTTON_PRESSED(button) ((currentButtons & button) && !(last.buttons & button))
            #define WAS_MISC_BUTTON_PRESSED(button) ((currentMiscButtons & button) && !(last.miscButtons & button))

            if (WAS_MAIN_BUTTON_PRESSED(XBOX_BTN_TOGGLE_HEADLIGHT)) {
                digitalWrite(PIN_PHARE, !digitalRead(PIN_PHARE));
                ctl->playDualRumble(0, 150, 0x80, 0x80);
            }

            if (WAS_MAIN_BUTTON_PRESSED(XBOX_BTN_TOGGLE_AVOIDANCE)) {
                if (robot_ptr->currentState == OBSTACLE_AVOIDANCE) changeState(*robot_ptr, IDLE);
                else changeState(*robot_ptr, OBSTACLE_AVOIDANCE);
            }

            if (WAS_MAIN_BUTTON_PRESSED(XBOX_BTN_TOGGLE_SENTRY)) {
                if (robot_ptr->currentState == SENTRY_MODE) changeState(*robot_ptr, IDLE);
                else changeState(*robot_ptr, SENTRY_MODE);
            }

            if (WAS_MAIN_BUTTON_PRESSED(XBOX_BTN_SPEED_UP)) {
                robot_ptr->vitesseCible = constrain(robot_ptr->vitesseCible + XBOX_SPEED_INCREMENT, 0, PWM_MAX);
            }

            if (WAS_MAIN_BUTTON_PRESSED(XBOX_BTN_SPEED_DOWN)) {
                robot_ptr->vitesseCible = constrain(robot_ptr->vitesseCible - XBOX_SPEED_INCREMENT, 0, PWM_MAX);
            }
            
            if (WAS_MISC_BUTTON_PRESSED(XBOX_BTN_EMERGENCY_STOP)) {
                changeState(*robot_ptr, IDLE);
            }

            uint8_t currentDpad = ctl->dpad();
            #define WAS_DPAD_PRESSED(button) ((currentDpad & button) && !(last.dpad & button))

            #if (XBOX_DPAD_UP_ACTION == XBOX_ACTION_CALIBRATE_COMPASS)
                if (WAS_DPAD_PRESSED(DPAD_UP)) changeState(*robot_ptr, CALIBRATING_COMPASS);
            #endif
            
            #undef WAS_MAIN_BUTTON_PRESSED
            #undef WAS_MISC_BUTTON_PRESSED
            #undef WAS_DPAD_PRESSED

            last.buttons = currentButtons;
            last.miscButtons = currentMiscButtons;
            last.dpad = currentDpad;
        }
    }
}
