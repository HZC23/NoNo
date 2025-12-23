#ifndef TERMINAL_H
#define TERMINAL_H

#include <Arduino.h>
#include <ctype.h>
#include "config.h"
#include "state.h"
#include "fonctions_motrices.h"
#include "compass.h"
#include "support.h" // For Phare control
#include <Preferences.h>   // For storing comm mode

extern Robot robot; // Use the global robot object from NoNo.ino
extern Preferences preferences; // Use the global preferences object from NoNo.ino

#if USB_MSC_ENABLED
extern bool usbMscActive;
#endif

// --- Defines for the command parser ---
#define CMD_DELIMITER ":"

// Forward declaration
void processCommand(String command);

/**
 * @brief Checks for incoming commands from the standard Serial port.
 * This should be called in the main loop.
 */
inline void checkSerial() {
    static char cmdBuffer[CMD_BUFFER_SIZE];
    static uint8_t cmdIndex = 0;

    while (Serial.available() > 0) {
        char incomingChar = Serial.read();

        if (incomingChar == '\n' || incomingChar == '\r') {
            if (cmdIndex > 0) {
                cmdBuffer[cmdIndex] = '\0'; // Null-terminate the string
                processCommand(String(cmdBuffer));
                cmdIndex = 0; // Reset for next command
            }
        } else if (cmdIndex < CMD_BUFFER_SIZE - 1) {
            cmdBuffer[cmdIndex++] = incomingChar;
        }
    }
}


#endif // TERMINAL_H
