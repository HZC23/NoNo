#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <ArduinoJson.h> // This library is required!
#include "state.h"
#include "support.h" // For readBatteryPercentage

// Sends the robot's current state as a JSON object over Serial
inline void sendTelemetry(Robot& robot) {
    // Use a static document to avoid heap allocation in the loop
    StaticJsonDocument<JSON_DOC_SIZE> doc;

    // Get the string representation of the current state
    const char* stateString;
    switch (robot.currentState) {
        case IDLE: stateString = "IDLE"; break;
        case MOVING_FORWARD:
        case MANUAL_FORWARD:
            stateString = "MOVING_FORWARD"; break;
        case MOVING_BACKWARD:
        case MANUAL_BACKWARD:
            stateString = "MOVING_BACKWARD"; break;
        case TURNING_LEFT:
        case MANUAL_TURNING_LEFT:
            stateString = "TURNING_LEFT"; break;
        case TURNING_RIGHT:
        case MANUAL_TURNING_RIGHT:
            stateString = "TURNING_RIGHT"; break;
        case FOLLOW_HEADING: stateString = "FOLLOW_HEADING"; break;
        case MAINTAIN_HEADING: stateString = "MAINTAIN_HEADING"; break;
        case OBSTACLE_AVOIDANCE: stateString = "OBSTACLE_AVOIDANCE"; break;
        case SCANNING: stateString = "SCANNING"; break;
        default: stateString = "UNKNOWN"; break;
    }

    // Populate the JSON document
    doc["state"] = stateString;
    doc["heading"] = robot.cap;
    doc["distance"] = robot.dusm;
    doc["distanceLaser"] = robot.distanceLaser;
    doc["battery"] = readBatteryPercentage(); // Use the real function now
    doc["speedTarget"] = robot.vitesseCible;

    // Serialize JSON to Serial port
    serializeJson(doc, Serial);
    Serial.println(); // Send a newline to indicate the end of the message
}

#endif // TELEMETRY_H