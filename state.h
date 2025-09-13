
#ifndef STATE_H
#define STATE_H

#include <Arduino.h>
#include <FS_MX1508.h>
#include <Servo.h>
#include <LSM303.h>
#include <DFRobot_RGBLCD1602.h>
#include "config.h"

// Structure to hold the robot's state
struct Robot {
    // State Machine
    RobotState currentState = IDLE;
    NavigationMode currentNavMode = MANUAL_CONTROL;
    unsigned long lastActionTime = 0;
    bool actionStarted = false;

    // Motion
    int vitesseCible = 0;
    int vitesseCourante = 0;
    bool hasReculed = false;
    bool hasTurned = false;

    // Navigation
    float capCibleRotation = 0;
    int Ncap = INITIAL_NCAP;
    int cap = INITIAL_CAP;

    // Manual Override
    int manualDistance = 0;
    int manualStartDistance = 0;
    bool manualMovementActive = false;

    // Sensors
    int dusm = 0; // Distance UltraSon Mesuree

    // Compass Calibration
    bool compassInitialized = false;
    bool compassCalibrated = false;
    bool compassInverted = false;
    float compassOffset = 180;
    LSM303::vector<int16_t> magMin = {32767, 32767, 32767};
    LSM303::vector<int16_t> magMax = {-32768, -32768, -32768};

    // LCD
    String lcdText = "";

    // Profiling
    unsigned long loopStartTime, loopEndTime;
    unsigned long terminalTime, sensorTaskTime, motorControlTime;
    unsigned long lastReportTime = 0;
    const unsigned long reportInterval = 2000;
};

// Declare hardware objects as extern so they can be used in other files
// The actual objects are defined in NoNo.ino
extern MX1508 motorA;
extern MX1508 motorB;
extern Servo Servodirection;
extern LSM303 compass;
extern DFRobot_RGBLCD1602 lcd;

#endif // STATE_H
