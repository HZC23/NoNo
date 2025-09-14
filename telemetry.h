#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <ArduinoJson.h> // This library is required!
#include "state.h"
#include "support.h" // For readBatteryPercentage

// Sends the robot's current state as a JSON object over Serial
inline void sendTelemetry(Robot& robot) {
    // Use a static document to avoid heap allocation in the loop
    StaticJsonDocument<200> doc;

    // Get the string representation of the current state
    const char* stateString;
    switch (robot.currentState) {
        case IDLE: stateString = "IDLE"; break;
        case MOVING_FORWARD: stateString = "MOVING_FORWARD"; break;
        case MOVING_BACKWARD: stateString = "MOVING_BACKWARD"; break;
        case TURNING_LEFT: stateString = "TURNING_LEFT"; break;
        case TURNING_RIGHT: stateString = "TURNING_RIGHT"; break;
        case FOLLOW_HEADING: stateString = "FOLLOW_HEADING"; break;
        default: stateString = "UNKNOWN"; break;
    }

    // Populate the JSON document
    doc["state"] = stateString;
    doc["heading"] = robot.cap;
    doc["distance"] = robot.dusm;
    doc["battery"] = readBatteryPercentage(); // Use the real function now
    doc["speedTarget"] = robot.vitesseCible;
    doc["speedCurrent"] = robot.vitesseCourante;

    // Serialize JSON to Serial port
    serializeJson(doc, Serial);
    Serial.println(); // Send a newline to indicate the end of the message
}

#endif // TELEMETRY_H