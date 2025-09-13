#ifndef TERMINAL_H
#define TERMINAL_H

#include <Arduino.h>
#include "state.h"
#include "fonctions_motrices.h" // For changeState, Arret
#include "compass.h" // For calibrateCompass

// This function is very large and complex.
// For this refactoring, we are just isolating it.
// A future improvement would be to break this function down into smaller pieces.
inline void Terminal(Robot& robot) {
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        command.trim();

        if (DEBUG_MODE) {
            Serial.print("Commande recue: ");
            Serial.println(command);
        }

        if (command.equalsIgnoreCase("stop")) {
            robot.vitesseCible = 0;
            changeState(robot, IDLE);
        } else if (command.equalsIgnoreCase("avant")) {
            robot.vitesseCible = VITESSE_MOYENNE;
            changeState(robot, MOVING_FORWARD);
        } else if (command.equalsIgnoreCase("arriere")) {
            robot.vitesseCible = VITESSE_LENTE;
            changeState(robot, MOVING_BACKWARD);
        } else if (command.startsWith("vitesse")) {
            int speed = command.substring(8).toInt();
            robot.vitesseCible = constrain(speed, 0, 255);
        } else if (command.startsWith("cap")) {
            int newCap = command.substring(4).toInt();
            robot.Ncap = newCap;
            changeState(robot, FOLLOW_HEADING);
        } else if (command.equalsIgnoreCase("calib")) {
            changeState(robot, CALIBRATING_COMPASS);
        }
        // Add other commands here...
    }
}

#endif // TERMINAL_H
