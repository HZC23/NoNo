#include "xbox_controller_bluepad.h"

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
    // This function should be called in the main loop()
    BP32.update();

    for (int i = 0; i < BP32_MAX_CONTROLLERS; i++) {
        ControllerPtr ctl = myControllers[i];
        if (ctl && ctl->isConnected() && ctl->hasData()) {
            
            // --- Left Joystick: Movement ---
            int32_t joyX = ctl->axisX();
            int32_t joyY = ctl->axisY();

            // Apply deadzone
            if (abs(joyX) < 150) joyX = 0;
            if (abs(joyY) < 150) joyY = 0;

            // Map joystick values to PWM range
            // Stick values are -1024 to 1023.
            int throttle = map(joyY, -1024, 1023, PWM_MAX, -PWM_MAX); // Invert Y
            int turn = map(joyX, -1024, 1023, -PWM_MAX, PWM_MAX);
            
            int pwmRight = constrain(throttle - turn, -PWM_MAX, PWM_MAX);
            int pwmLeft = constrain(throttle + turn, -PWM_MAX, PWM_MAX);

            // Only send motor commands if the robot's state is explicitly set to APP_CONTROL.
            // This prevents the controller from interfering with autonomous modes like OBSTACLE_AVOIDANCE.
            if (robot_ptr->currentState == APP_CONTROL) {
                motorA_ptr->motorGo(pwmRight);
                motorB_ptr->motorGo(pwmLeft);
            }

            // State management: switch to APP_CONTROL on joystick input, or back to IDLE when input stops.
            if (pwmLeft != 0 || pwmRight != 0) {
                changeState(*robot_ptr, APP_CONTROL);
            } else if (robot_ptr->currentState == APP_CONTROL) {
                changeState(*robot_ptr, IDLE);
            }

            // --- Right Joystick: Turret Control ---
            int32_t turretX = ctl->axisRX();
            int32_t turretY = ctl->axisRY();

            if (abs(turretX) > 150 || abs(turretY) > 150) {
                int targetHAngle = map(turretX, -1024, 1023, SCAN_H_END_ANGLE, SCAN_H_START_ANGLE);
                int targetVAngle = map(turretY, 1024, -1023, SCAN_V_END_ANGLE, SCAN_V_START_ANGLE); // Invert Y
                tourelle_ptr->write(targetHAngle, targetVAngle);
            }

            // --- Button Mappings (single press detection) ---
            ControllerState& last = lastStates[i];

            if (ctl->a() && !last.a) {
                digitalWrite(PIN_PHARE, !digitalRead(PIN_PHARE));
                ctl->playDualRumble(0, 150, 0x80, 0x80); // Use playDualRumble
            }
            if (ctl->b() && !last.b) {
                if (robot_ptr->currentState == OBSTACLE_AVOIDANCE) {
                    changeState(*robot_ptr, IDLE);
                } else {
                    changeState(*robot_ptr, OBSTACLE_AVOIDANCE);
                }
            }
            if (ctl->x() && !last.x) {
                if (robot_ptr->currentState == SENTRY_MODE) {
                    changeState(*robot_ptr, IDLE);
                } else {
                    changeState(*robot_ptr, SENTRY_MODE);
                }
            }
            if (ctl->y() && !last.y) {
                changeState(*robot_ptr, SCANNING_3D);
            }
            // Use bitmask for shoulder buttons
            if ((ctl->buttons() & BUTTON_SHOULDER_R) && !(last.buttons & BUTTON_SHOULDER_R)) {
                 robot_ptr->vitesseCible = constrain(robot_ptr->vitesseCible + XBOX_SPEED_INCREMENT, 0, PWM_MAX);
            }
            if ((ctl->buttons() & BUTTON_SHOULDER_L) && !(last.buttons & BUTTON_SHOULDER_L)) {
                robot_ptr->vitesseCible = constrain(robot_ptr->vitesseCible - XBOX_SPEED_INCREMENT, 0, PWM_MAX);
            }
            
            uint16_t misc = ctl->miscButtons();
            // MISC_BUTTON_BACK == "View" button
            if ((misc & MISC_BUTTON_BACK) && !(last.miscButtons & MISC_BUTTON_BACK)) {
                 tourelle_ptr->write(SCAN_CENTER_ANGLE, NEUTRE_TOURELLE);
            }
            // MISC_BUTTON_HOME == "Menu" button
            if ((misc & MISC_BUTTON_HOME) && !(last.miscButtons & MISC_BUTTON_HOME)) {
                changeState(*robot_ptr, IDLE);
                setLcdText(*robot_ptr, "Xbox OFF");
            }


            // --- Update last state for next iteration ---
            last.a = ctl->a();
            last.b = ctl->b();
            last.x = ctl->x();
            last.y = ctl->y();
            last.buttons = ctl->buttons(); // Store full button state
            last.miscButtons = misc;
            last.dpad = ctl->dpad();
        }
    }
}