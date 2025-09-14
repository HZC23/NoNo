#ifndef TERMINAL_H
#define TERMINAL_H

#include <Arduino.h>
#include "state.h"
#include "fonctions_motrices.h"
#include "compass.h"
#include "support.h" // For Phare control

// Parses and executes commands received from the Serial port
inline void Terminal(Robot& robot) {
    if (!Serial.available()) {
        return;
    }

    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command.length() == 0) {
        return;
    }

    if (DEBUG_MODE) {
        Serial.print("Command received: [");
        Serial.print(command);
        Serial.println("]");
    }

    // Temporary variables for parsing
    String type, action, value;
    int firstColon = command.indexOf(':');
    int secondColon = command.indexOf(':', firstColon + 1);

    // Parse the command string "TYPE:ACTION:VALUE"
    if (firstColon > 0 && secondColon > firstColon) {
        type = command.substring(0, firstColon);
        action = command.substring(firstColon + 1, secondColon);
        value = command.substring(secondColon + 1);
    } else {
        if (DEBUG_MODE) Serial.println("Error: Invalid command format");
        return;
    }

    // Process commands based on type
    if (type.equalsIgnoreCase("CMD")) {
        if (action.equalsIgnoreCase("MOVE")) {
            if (value.equalsIgnoreCase("FWD")) {
                robot.vitesseCible = VITESSE_MOYENNE; // Use a default speed
                changeState(robot, MOVING_FORWARD);
            } else if (value.equalsIgnoreCase("BWD")) {
                robot.vitesseCible = VITESSE_LENTE;
                changeState(robot, MOVING_BACKWARD);
            } else if (value.equalsIgnoreCase("LEFT")) {
                changeState(robot, MANUAL_TURNING_LEFT);
            } else if (value.equalsIgnoreCase("RIGHT")) {
                changeState(robot, MANUAL_TURNING_RIGHT);
            } else if (value.equalsIgnoreCase("STOP")) {
                robot.vitesseCible = 0;
                changeState(robot, IDLE);
            }
        } else if (action.equalsIgnoreCase("SPEED")) {
            robot.vitesseCible = constrain(value.toInt(), 0, 255);
            if (DEBUG_MODE) {
                Serial.print("Vitesse cible mise a: ");
                Serial.println(robot.vitesseCible);
            }
        } else if (action.equalsIgnoreCase("GOTO")) {
            robot.Ncap = value.toInt();
            robot.vitesseCible = VITESSE_MOYENNE;
            changeState(robot, FOLLOW_HEADING);
        } else if (action.equalsIgnoreCase("TURN")) {
            robot.capCibleRotation = value.toFloat();
            changeState(robot, TURNING_LEFT); // Or right based on error, handled in motor control
        } else if (action.equalsIgnoreCase("LIGHT")) {
            if (value.equalsIgnoreCase("ON")) {
                PhareAllume();
            } else if (value.equalsIgnoreCase("OFF")) {
                PhareEteint();
            }
        } else if (action.equalsIgnoreCase("CALIBRATE")) {
            if (value.equalsIgnoreCase("COMPASS")) {
                changeState(robot, CALIBRATING_COMPASS);
            }
        }
    }
}

#endif // TERMINAL_H
