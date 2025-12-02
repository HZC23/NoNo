#ifndef STATE_H
#define STATE_H

#include <Arduino.h>
#include <FS_MX1508.h>
#include <Servo.h>
#include <LSM303.h>
#include <DFRobot_RGBLCD1602.h>
#include "config.h"
#include "tourelle.h" // Added for Tourelle class

enum ObstacleAvoidanceState {
  AVOID_START,
  AVOID_QUICK_SCAN_LEFT,
  AVOID_WAIT_FOR_LEFT_SCAN,
  AVOID_QUICK_SCAN_RIGHT,
  AVOID_WAIT_FOR_RIGHT_SCAN,
  AVOID_CENTER_TURRET,
  AVOID_WAIT_FOR_CENTER,
  AVOID_FULL_SCAN_START,
  AVOID_FULL_SCAN_STEP,
  AVOID_FULL_SCAN_FINISH,
  AVOID_TURN_TO_BEST_ANGLE,
  AVOID_BACKUP
};

enum GroundCheckState {
  GC_START,
  GC_LOOK_DOWN,
  GC_WAIT,
  GC_CHECK,
  GC_LOOK_UP,
  GC_FINISH
};

enum HeadAnimationType { ANIM_NONE, ANIM_SHAKE_NO, ANIM_NOD_YES };

// Structure to hold the robot's state
struct Robot {
    // State Machine
    RobotState currentState = IDLE;
    ObstacleAvoidanceState obstacleAvoidanceState = AVOID_START;
    GroundCheckState groundCheckState = GC_START;
    RobotState stateBeforeGroundCheck = IDLE;

    HeadAnimationType currentHeadAnimation = ANIM_NONE;
    unsigned long headAnimStartTime = 0;
    int headAnimCycles = 0; // Number of times to repeat the animation (e.g., 1 for a single nod, 3 for three shakes)
    RobotState stateBeforeHeadAnimation = IDLE; // Store the state before starting an animation

    NavigationMode currentNavMode = MANUAL_CONTROL;
    unsigned long lastActionTime = 0;
    bool actionStarted = false;
    bool initialActionTaken = false;
    int consecutiveAvoidManeuvers = 0;
    unsigned long turretMoveStartTime = 0;
    RobotState nextStateAfterTurretMove = IDLE;

    // Motion
    int vitesseCible = 0;
    int speedAvg = VITESSE_MOYENNE; // Default average speed
    int speedSlow = VITESSE_LENTE;   // Default slow speed
    bool hasReculed = false;
    bool controlInverted = false; // Flag for control inversion
    bool hasTurned = false;

    // Navigation
    float capCibleRotation = 0;
    int Ncap = INITIAL_NCAP;
    int cap = INITIAL_CAP;

    // Battery
    bool batteryIsLow = false;
    bool batteryIsCritical = false;

    // Manual Override
    int manualDistance = 0;
    int manualStartDistance = 0;
    bool manualMovementActive = false;

    // Sensors
    int dusm = 0; // Distance UltraSon Mesuree
    int distanceLaser = 0;
    bool obstacleDetectedByLaser = false;
    bool laserInitialized = false;

    // Cliff Detection
    unsigned long lastCliffCheckTime = 0;
    
    // Horizon Stabilization
    float currentPitch = 0.0;

    // Scanning
    int currentScanAngleH = SCAN_H_START_ANGLE;
    // int currentScanAngleV = SCAN_V_START_ANGLE;
    unsigned long lastScanTime = 0;
    int scanDistances[SCAN_DISTANCE_ARRAY_SIZE]; // To store distances for angles 0-180
    int bestAvoidAngle;

    // Compass Calibration
    bool compassInitialized = false;
    bool compassCalibrated = false;
    bool compassInverted = false;
    float compassOffset = 0.0;
    LSM303::vector<int16_t> magMin = {32767, 32767, 32767};
    LSM303::vector<int16_t> magMax = {-32768, -32768, -32768};

    // LCD
    char lcdText[MAX_LCD_TEXT_LENGTH + 1];
    unsigned long lastLcdUpdateTime = 0;
    unsigned long lastJokeDisplayTime = 0;
    char musicFileName[64]; // To store the name of the music file to play
    int currentPage;

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
extern Tourelle tourelle; // Added for Tourelle control
extern VL53L1X vl53;

#endif // STATE_H