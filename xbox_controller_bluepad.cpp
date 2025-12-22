#include "xbox_controller_bluepad.h"
#include "terminal.h" // For processCommand

// Struct to hold the previous state of a controller for single-press detection
struct ControllerState {
    uint8_t dpad = 0;
    uint16_t miscButtons = 0;
    uint32_t buttons = 0; // To store the full button bitmask
    bool a = false, b = false, x = false, y = false;
    bool thumbL = false, thumbR = false;
};

// --- Initialize Static Members ---
Robot* XboxControllerBluepad::robot_ptr = nullptr;
MX1508* XboxControllerBluepad::motorA_ptr = nullptr;
MX1508* XboxControllerBluepad::motorB_ptr = nullptr;
Tourelle* XboxControllerBluepad::tourelle_ptr = nullptr;
ControllerPtr XboxControllerBluepad::myControllers[BP32_MAX_CONTROLLERS];
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
                // Reset last state
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
    // Setup the Bluepad32 callbacks
    BP32.setup(&XboxControllerBluepad::onConnectedController, &XboxControllerBluepad::onDisconnectedController);
    // You might need to call BP32.forgetBluetoothKeys() if you have connection issues
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
            
            // --- Left Joystick: Movement ---
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

            // --- Right Joystick: Turret Control (REMOVED) ---

            // --- Configurable Button Mappings ---
            uint32_t currentButtons = ctl->buttons();
            uint16_t currentMiscButtons = ctl->miscButtons();
            ControllerState& last = lastStates[i];

            // Helper macros for detecting a single press
            #define WAS_MAIN_BUTTON_PRESSED(button) ((currentButtons & button) && !(last.buttons & button))
            #define WAS_MISC_BUTTON_PRESSED(button) ((currentMiscButtons & button) && !(last.miscButtons & button))

            // Action: Toggle Headlight
            if (WAS_MAIN_BUTTON_PRESSED(XBOX_BTN_TOGGLE_HEADLIGHT)) {
                digitalWrite(PIN_PHARE, !digitalRead(PIN_PHARE));
                ctl->playDualRumble(0, 150, 0x80, 0x80);
            }

            // Action: Toggle Obstacle Avoidance
            if (WAS_MAIN_BUTTON_PRESSED(XBOX_BTN_TOGGLE_AVOIDANCE)) {
                if (robot_ptr->currentState == OBSTACLE_AVOIDANCE) {
                    changeState(*robot_ptr, IDLE);
                } else {
                    changeState(*robot_ptr, OBSTACLE_AVOIDANCE);
                }
            }

            // Action: Toggle Sentry Mode
            if (WAS_MAIN_BUTTON_PRESSED(XBOX_BTN_TOGGLE_SENTRY)) {
                if (robot_ptr->currentState == SENTRY_MODE) {
                    changeState(*robot_ptr, IDLE);
                } else {
                    changeState(*robot_ptr, SENTRY_MODE);
                }
            }

            // Action: Speed Up
            if (WAS_MAIN_BUTTON_PRESSED(XBOX_BTN_SPEED_UP)) {
                robot_ptr->vitesseCible = constrain(robot_ptr->vitesseCible + XBOX_SPEED_INCREMENT, 0, PWM_MAX);
            }

            // Action: Speed Down
            if (WAS_MAIN_BUTTON_PRESSED(XBOX_BTN_SPEED_DOWN)) {
                robot_ptr->vitesseCible = constrain(robot_ptr->vitesseCible - XBOX_SPEED_INCREMENT, 0, PWM_MAX);
            }
            
            // Action: Emergency Stop (sets state to IDLE)
            if (WAS_MISC_BUTTON_PRESSED(XBOX_BTN_EMERGENCY_STOP)) {
                changeState(*robot_ptr, IDLE);
            }

            // --- D-Pad Actions ---
            uint8_t currentDpad = ctl->dpad();
            #define WAS_DPAD_PRESSED(button) ((currentDpad & button) && !(last.dpad & button))

            #if (XBOX_DPAD_UP_ACTION == XBOX_ACTION_CALIBRATE_COMPASS)
                if (WAS_DPAD_PRESSED(DPAD_UP)) changeState(*robot_ptr, CALIBRATING_COMPASS);
            #endif

            #if (XBOX_DPAD_DOWN_ACTION == XBOX_ACTION_CALIBRATE_COMPASS)
                if (WAS_DPAD_PRESSED(DPAD_DOWN)) changeState(*robot_ptr, CALIBRATING_COMPASS);
            #endif

            #if (XBOX_DPAD_LEFT_ACTION == XBOX_ACTION_CALIBRATE_COMPASS)
                if (WAS_DPAD_PRESSED(DPAD_LEFT)) changeState(*robot_ptr, CALIBRATING_COMPASS);
            #endif

            #if (XBOX_DPAD_RIGHT_ACTION == XBOX_ACTION_CALIBRATE_COMPASS)
                if (WAS_DPAD_PRESSED(DPAD_RIGHT)) changeState(*robot_ptr, CALIBRATING_COMPASS);
            #endif
            
            // Cleanup the helper macros
            #undef WAS_MAIN_BUTTON_PRESSED
            #undef WAS_MISC_BUTTON_PRESSED
            #undef WAS_DPAD_PRESSED

            // --- Update last state for next iteration ---
            last.buttons = currentButtons;
            last.miscButtons = currentMiscButtons;
            last.dpad = currentDpad;
        }
    }
}