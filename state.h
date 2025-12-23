#ifndef STATE_H
#define STATE_H

#include <Arduino.h>
#include "config.h"

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
  AVOID_BACKUP,
  AVOID_TURN_IN_PLACE,    // New state: turn in place to escape tight spots
  AVOID_FINISH_TURN       // New state: wait for turn to complete
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

enum SentryState {
  SENTRY_IDLE,
  SENTRY_SCAN_START,
  SENTRY_SCAN_STEP,
  SENTRY_TRACKING,
  SENTRY_ALARM
};

enum CommunicationMode {
  COMM_MODE_SERIAL,
  COMM_MODE_XBOX
};

// Enum for the sub-states of the emergency evasion maneuver
enum EvasionState {
  EVADE_START,
  EVADE_BACKUP,
  EVADE_PIVOT,
  EVADE_FINISH
};

// Structure to hold the robot's state
struct Robot {
    // State Machine
    RobotState currentState = IDLE;
    ObstacleAvoidanceState obstacleAvoidanceState = AVOID_START;
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
    int speedAvg = VITESSE_MOYENNE; // Default average speed
    int speedSlow = VITESSE_LENTE;   // Default slow speed
    int initialSpeedAvg = VITESSE_MOYENNE; // Stores the initial average speed (from config or default)
    int initialSpeedSlow = VITESSE_LENTE;   // Stores the initial slow speed (from config or default)
    int speedRotation = VITESSE_ROTATION; // Default rotation speed, loaded from config
    float turnTolerance = TOLERANCE_VIRAGE; // Default turn tolerance, loaded from config
    float KpHeading = Kp_HEADING; // Default Kp for heading, loaded from config
    int speedRotationMax = VITESSE_ROTATION_MAX; // Max rotation speed, loaded from config
    bool hasReculed = false;
    bool controlInverted = false; // Flag for control inversion
    bool hasTurned = false;
    // isStuckConfirmed: Flag set to true when the robot detects it is physically stuck (e.g., against a wall)
    //                   Used for traction control and escape maneuvers.
    bool isStuckConfirmed = false;
    // currentSteeringBias: Stores the current steering angle relative to center (e.g., -30 to +30 degrees).
    //                      Used by power steering to provide more power during sharp turns.
    int currentSteeringBias = 0;
    float pivotAngleThreshold = SEUIL_BASCULE_DIRECTION;

    // Motion Physics
    float motorBCalibration = CALIBRATION_MOTEUR_B;
    int minSpeedToMove = MIN_SPEED_TO_MOVE;
    float accelRate = ACCEL_RATE;
    float diffStrength = DIFF_STRENGTH;
    float fwdDiffCoeff = FWD_DIFF_COEFF;


    // Navigation
    float capCibleRotation = 0;
    int Ncap = INITIAL_NCAP;
    int cap = INITIAL_CAP;

    // Servos
    int servoNeutralDir = NEUTRE_DIRECTION;
    int servoNeutralTurret = NEUTRE_TOURELLE;
    int servoDirMin = SERVO_DIR_MIN;
    int servoDirMax = SERVO_DIR_MAX;
    int servoAngleHeadDown = ANGLE_TETE_BASSE;
    int servoAngleGround = ANGLE_SOL;

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
    unsigned long laserTimingBudget = VL53L1X_TIMING_BUDGET_US;
    unsigned long laserInterMeasurementPeriod = VL53L1X_INTER_MEASUREMENT_PERIOD_MS;
    int maxUltrasonicDistance = MAX_ULTRASONIC_DISTANCE;


    // Cliff Detection
    unsigned long lastCliffCheckTime = 0;
    int seuilVide = SEUIL_VIDE;
    
    // Horizon Stabilization
    float currentPitch = 0.0;

    // Sentry Mode
    bool lastPIRState = LOW;
    int intruderAngle = 0;

    // Curious Mode (Idle Animation)
    int curiousTargetH = SCAN_CENTER_ANGLE;
    int curiousTargetV = NEUTRE_TOURELLE;

    // Scanning
    int currentScanAngleH = SCAN_H_START_ANGLE;
    int currentScanAngleV = 90;
    unsigned long lastScanTime = 0;
    int scanDistances[SCAN_DISTANCE_ARRAY_SIZE]; // To store distances for angles 0-180
    int bestAvoidAngle;
    float anglePenaltyFactor = ANGLE_PENALTY_FACTOR;

    // Turret
    unsigned long turretMoveTime = TURRET_MOVE_TIME_MS;
    unsigned long turretScanDelay = SCAN_DELAY_MS;

    // Obstacle Avoidance
    unsigned long avoidBackupDuration = AVOID_BACKUP_DURATION_MS;
    int minDistForValidPath = MIN_DIST_FOR_VALID_PATH;

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
    unsigned long initialAutonomousDelay = INITIAL_AUTONOMOUS_DELAY_MS;
};



#endif // STATE_H