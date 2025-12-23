#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <ArduinoJson.h>
#include "state.h"
#include "support.h"

// Sends the robot's current state as a JSON object
inline void sendTelemetry(Robot& robot) {
    JsonDocument doc;

    // Get the string representation of the current state
    const char* stateString = stateToString(robot.currentState);

    // Populate the JSON document
    doc["state"] = stateString;
    doc["heading"] = robot.cap;
    doc["distance"] = robot.dusm;
    doc["distanceLaser"] = robot.distanceLaser;
    doc["battery"] = readBatteryPercentage();
    doc["speedTarget"] = robot.vitesseCible;

    // 2. Always send to Serial for debugging, unless specifically disabled
    serializeJson(doc, Serial);
    Serial.println();
}

#endif // TELEMETRY_H