#ifndef ROBOT_H
#define ROBOT_H

#include <Arduino.h>
#include "config.h"
#include <LSM303.h> // Required for the vector type in the Robot struct

// Structure to hold the robot's state
struct Robot {
    // State Machine
    RobotState currentState;
    ObstacleAvoidanceState obstacleAvoidanceState;
    EvasionState evasionState = EVADE_START;
    GroundCheckState groundCheckState = GC_START;
    SentryState sentryState = SENTRY_IDLE;
    RobotState stateBeforeGroundCheck = IDLE;

    HeadAnimationType currentHeadAnimation = ANIM_NONE;
    unsigned long headAnimStartTime = 0;
    int headAnimCycles = 0; // Number of times to repeat the animation (e.g., 1 for a single nod, 3 for three shakes)
    RobotState stateBeforeHeadAnimation = IDLE; // Store the state before starting an animation

    NavigationMode currentNavMode = MANUAL_CONTROL;
    CommunicationMode activeCommMode = COMM_MODE_SERIAL;
    unsigned long lastActionTime = 0;
    unsigned long lastAppCommandTime = 0;
    bool actionStarted = false;
    bool initialActionTaken = false;
    int consecutiveAvoidManeuvers = 0;
    unsigned long turretMoveStartTime = 0;
    RobotState nextStateAfterTurretMove = IDLE;

    // Motion
    int vitesseCible = 0;
    int speedAvg; // Default average speed
    int speedSlow;   // Default slow speed
    int initialSpeedAvg; // Stores the initial average speed (from config or default)
    int initialSpeedSlow;   // Stores the initial slow speed (from config or default)
    int speedRotation; // Default rotation speed, loaded from config
    float turnTolerance; // Default turn tolerance, loaded from config
    float KpHeading; // Default Kp for heading, loaded from config
    int speedRotationMax; // Max rotation speed, loaded from config
    bool hasReculed = false;
    bool controlInverted = false; // Flag for control inversion
    bool hasTurned = false;
    // isStuckConfirmed: Flag set to true when the robot detects it is physically stuck (e.g., against a wall)
    //                   Used for traction control and escape maneuvers.
    bool isStuckConfirmed = false;
    // currentSteeringBias: Stores the current steering angle relative to center (e.g., -30 to +30 degrees).
    //                      Used by power steering to provide more power during sharp turns.
    int currentSteeringBias;
    float pivotAngleThreshold;

    // Motion Physics
    float motorBCalibration;
    int minSpeedToMove;
    float accelRate;
    float diffStrength;
    float fwdDiffCoeff;
    float currentPwmA = 0;
    float currentPwmB = 0;



    // Navigation
    float capCibleRotation;
    int Ncap;
    int cap;

    // Servos
    int servoNeutralDir;
    int servoNeutralTurret;
    int servoDirMin;
    int servoDirMax;
    int servoAngleHeadDown;
    int servoAngleGround;

    // Battery
    bool batteryIsLow = false;
    bool batteryIsCritical = false;

    // SD Card
    bool sdCardReady = false;

    // Manual Override
    int manualDistance = 0;
    int manualStartDistance = 0;
    bool manualMovementActive = false;

    // Sensors
    int dusm = 0; // Distance UltraSon Mesuree
    int distanceLaser = 0;
    bool obstacleDetectedByLaser = false;
    bool laserInitialized = false;
    unsigned long laserTimingBudget;
    unsigned long laserInterMeasurementPeriod;
    int maxUltrasonicDistance;


    // Cliff Detection
    unsigned long lastCliffCheckTime;
    int seuilVide;
    
    // Horizon Stabilization
    float currentPitch = 0.0;

    // Sentry Mode
    bool lastPIRState = LOW;
    int intruderAngle = 0;

    // Curious Mode (Idle Animation)
    int curiousTargetH;
    int curiousTargetV;

    // Scanning
    int currentScanAngleH;
    int currentScanAngleV = 90;
    unsigned long lastScanTime;
    int scanDistances[SCAN_DISTANCE_ARRAY_SIZE]; // To store distances for angles 0-180
    int bestAvoidAngle;
    float anglePenaltyFactor;

    // Turret
    unsigned long turretMoveTime;
    unsigned long turretScanDelay;

    // Obstacle Avoidance
    unsigned long avoidBackupDuration;
    int minDistForValidPath;

    // Compass Calibration
    bool compassInitialized = false;
    bool compassCalibrated = false;
    bool compassInverted = false;
    float compassOffset = 0.0;
    LSM303::vector<int16_t> magMin = {32767, 32767, 32767};
    LSM303::vector<int16_t> magMax = {-32768, -32768, -32768};

// LCD
    char lcdText[MAX_LCD_TEXT_LENGTH + 1];
    char lcdFormattedText[128];                  // Text after word-wrap has been applied
    
    enum LcdAnimationState { ANIM_IDLE, ANIM_TYPEWRITER, ANIM_SCROLL_PAUSE };
    LcdAnimationState lcdAnimationState = ANIM_IDLE; // Manages the animation sequence

    unsigned long lcdAnimationNextCharTime = 0;  // Time to print the next character
    int lcdAnimationIndex = 0;                   // Current character index in lcdFormattedText
    int lcdCursorX = 0;                          // Current cursor column on the LCD
    int lcdCursorY = 0;                          // Current cursor row on the LCD

    unsigned long lastLcdUpdateTime = 0;
    unsigned long customMessageSetTime = 0; // Time when a custom message was set
    unsigned long lastJokeDisplayTime = 0;
    int currentPage;

    // Profiling
    unsigned long loopStartTime, loopEndTime;
    unsigned long terminalTime, sensorTaskTime, motorControlTime;
    unsigned long lastReportTime = 0;
    const unsigned long reportInterval = 2000;

    // Initialization
    unsigned long startTime;
    unsigned long initialAutonomousDelay;
    unsigned long lastCompassReadTime;
};

// --- Global Robot Instance ---
extern Robot robot;


#endif // ROBOT_H
