#ifndef CONFIG_H
#define CONFIG_H

// === DEBUG MODE ===
#define DEBUG_MODE true // Master switch for serial debug output

// ====================================================
// === CONFIGURATION ESP32-S3 LITE ===

// --- CARTE SD (Bus SPI1) ---
#define SD_CS_PIN     41 // Changed from 10
#define SD_SCK        39 // Changed from 12
#define SD_MISO       40 // Changed from 13
#define SD_MOSI       38 // Changed from 11

// --- BUS I2C ---
#define SDA_PIN       10
#define SCL_PIN       9

// --- SERVOMOTEURS (N'importe quelle pin PWM) ---
#define PINDIRECTION  1   // Servo Direction
#define PINTOURELLE_H 4   // Pan
#define PINTOURELLE_V 2   // Tilt

// --- MOTEURS DC (Pont en H) ---
#define AIN1          17
#define AIN2          18
#define BIN1          8
#define BIN2          16

// --- CAPTEURS ---
#define TRIGGER       6 // Moved from 38
#define ECHO          7 // Moved from 39
#define VBAT          5    // Entrée Analogique
#define PIR           14 // Moved from 40
#define INTERUPTPIN   21   // Bumper switch for collision detection

// --- SORTIES ---
#define NEOPIXEL_PIN  48   // Souvent la LED intégrée sur le S3
#define NEOPIXEL_COUNT 4
#define BUZZER_PIN    42
#define PIN_PHARE     45
// ====================================================

// --- Speed Settings ---
#define VITESSE_MOYENNE 200         // Standard forward speed (PWM 0-255)
#define VITESSE_LENTE 150           // Reduced speed for precision maneuvers
#define VITESSE_ROTATION 200        // Speed applied during in-place rotations
#define VITESSE_ROTATION_MAX 200    // Maximum allowable rotation speed
#define MIN_SPEED_TO_MOVE 60        // Minimum PWM value required to overcome friction
// --- Navigation Settings ---
#define TOLERANCE_VIRAGE 2.0        // Acceptable heading error in degrees
#define KP_HEADING 1.5              // Proportional gain for heading correction (steering sensitivity)
#define SEUIL_BASCULE_DIRECTION 30.0 // Angle threshold to switch between steering and pivoting
// --- Servo Angles ---
#define NEUTRE_DIRECTION 85         // Servo angle for straight steering (center)
#define NEUTRE_TOURELLE 90          // Servo angle for centered turret
#define SERVO_DIR_MIN 8             // Minimum mechanical limit for steering servo
#define SERVO_DIR_MAX 172           // Maximum mechanical limit for steering servo
#define ANGLE_TETE_BASSE 120        // Angle for "head down" position
#define ANGLE_SOL 140               // Angle for looking directly at the ground

// --- Motor Physics ---
#define CALIBRATION_MOTEUR_B 1.0    // Multiplier to balance Motor B speed relative to Motor A
#define ACCEL_RATE 0.15             // Acceleration smoothing factor
#define DIFF_STRENGTH 1.2           // Strength of differential steering effect
#define FWD_DIFF_COEFF 1.4          // Coefficient for differential steering while moving forward

// --- Obstacle Avoidance ---
#define AVOID_BACKUP_DURATION_MS 1000 // Time (ms) to reverse when avoiding an obstacle
#define MIN_DIST_FOR_VALID_PATH 40    // Minimum distance (cm) required to consider a path valid

// --- Turret ---
#define TURRET_MOVE_TIME_MS 300     // Duration (ms) allocated for turret movement

// --- Cliff Detection ---
#define SEUIL_VIDE 50               // Sensor threshold to detect a drop-off/cliff

// --- Laser Sensor ---
#define VL53L1X_TIMING_BUDGET_US 50000          // Time budget (us) for laser sensor measurement
#define VL53L1X_INTER_MEASUREMENT_PERIOD_MS 50  // Interval (ms) between laser measurements
// --- Initialization ---
#define INITIAL_AUTONOMOUS_DELAY_MS 10000       // Delay (ms) before autonomous mode starts
// === COMPASS CONSTANTS ===
#define COMPASS_READ_INTERVAL_MS 50
#define COMPASS_CALIBRATION_DURATION_MS 15000
#define COMPASS_CALIBRATION_VALIDATION_THRESHOLD 32000
#define COMPASS_MIN_INT16 -32768
#define COMPASS_MAX_INT16 32767
#define CARDINAL_NORTH_LOWER 337.5
#define CARDINAL_NORTH_UPPER 22.5
#define CARDINAL_NORTHEAST_UPPER 67.5
#define CARDINAL_EAST_UPPER 112.5
#define CARDINAL_SOUTHEAST_UPPER 157.5
#define CARDINAL_SOUTH_UPPER 202.5
#define CARDINAL_SOUTHWEST_UPPER 247.5
#define CARDINAL_WEST_UPPER 292.5

// === BATTERY CONFIGURATION ===
#define CRITICAL_BATTERY_LEVEL 1
#define BATTERY_TYPE_LIPO 0
#define BATTERY_TYPE_NIMH 1

// Select your battery type here
#define SELECTED_BATTERY_TYPE 1 // 0 for LiPo, 1 for NiMH
// Voltage for 2S LiPo
#define LIPO_MAX_VOLTAGE 8.4
#define LIPO_MIN_VOLTAGE 6.0

// Voltage for 6-cell NiMH (7.2V nominal)
#define NIMH_MAX_VOLTAGE 8.4 // 1.4V per cell, fully charged
#define NIMH_MIN_VOLTAGE 6.0 // 1.0V per cell, discharged

#define LOW_BATTERY_THRESHOLD 20 // %
#define RECHARGED_THRESHOLD 30   // % Threshold to exit low battery state (hysteresis)
#define ANGLE_TETE_BASSE 120 // "Sad" head angle


#define SERIAL_BAUD_RATE 115200
#define LCD_I2C_ADDR 0x60
#define LCD_ROWS 2

#define LCD_LINE_LENGTH 16
#define JSON_DOC_SIZE 200
#define CMD_BUFFER_SIZE 64
#define MAX_LCD_TEXT_LENGTH 64
#define SCROLL_DELAY_MS     3000 // The display time of each page (in ms)
#define LCD_IDLE_TIMEOUT_MS 5000 // 5 seconds of inactivity before jokes start
#define LCD_JOKE_INTERVAL_MS 5000 // Change joke every 5 seconds if still idle

// === INITIALIZATION ===
#define INITIAL_AUTONOMOUS_DELAY_MS 10000 // Delay before initial autonomous action at startup
#define MAX_CONSECUTIVE_AVOID_MANEUVERS 3

// === INITIALIZATION ===


// === LCD MESSAGES ===
#define LCD_TYPEWRITER_DELAY_MS 50
#define LCD_STARTUP_MESSAGE_1 "Je suis Nono"
#define LCD_STARTUP_MESSAGE_2 "Paré à exploser"
#define LCD_STARTUP_MESSAGE_3 "...explorer pardon"


// === HEAD ANIMATIONS ===
#define HEAD_SHAKE_NO_ANGLE_EXTREME_LEFT 135
#define HEAD_SHAKE_NO_ANGLE_EXTREME_RIGHT 45
#define HEAD_SHAKE_NO_CYCLE_DURATION_MS 200
#define HEAD_NOD_YES_ANGLE_UP 70
#define HEAD_NOD_YES_ANGLE_DOWN 110
#define HEAD_NOD_YES_CYCLE_DURATION_MS 200

// === CLIFF DETECTION ===
#define ANGLE_SOL 140 // Angle to look down at the ground (to be calibrated)
#define SEUIL_VIDE 50 // If ground is further than 50cm, it's a cliff
#define CLIFF_CHECK_INTERVAL_MS 500 // Check for cliff every 500ms when moving forward

// === LOOP REGULATOR ===
#define LOOP_TARGET_FREQUENCY 100 // Target frequency for the main loop in Hz
#define LOOP_TARGET_PERIOD_MS (1000 / LOOP_TARGET_FREQUENCY)

// === STATE & MODE DEFINITIONS ===
enum RobotState {
  IDLE,
  MOVING_FORWARD,
  MOVING_BACKWARD,
  TURNING_LEFT,
  TURNING_RIGHT,
  MANUAL_FORWARD,
  MANUAL_BACKWARD,
  MANUAL_TURNING_LEFT,
  MANUAL_TURNING_RIGHT,
  OBSTACLE_AVOIDANCE,
  WAITING_FOR_TURRET,
  FOLLOW_HEADING,
  MAINTAIN_HEADING,
  
  BACKING_UP_OBSTACLE, // New state for backing up from obstacle
  SCANNING_FOR_PATH,   // New state for scanning for a new path
  TURNING_TO_PATH,     // New state for turning to the new path
  SMART_TURNING,
  CALIBRATING_COMPASS,
  SMART_AVOIDANCE,
  SENTRY_MODE,
  CHECKING_GROUND,
  CLIFF_DETECTED,
  ANIMATING_HEAD,
  GAMEPAD_CONTROL,
  EMERGENCY_EVASION, // State for bumper-triggered escape maneuver
  STUCK                // State for when stall is detected
} ;

enum NavigationMode {
  MANUAL_CONTROL,
  AUTONOMOUS_CONTROL
};

// === COMMUNICATION MODE ===
// The communication mode is now defined as an enum in state.h and set at runtime.

// Set the active communication mode here
// #define ACTIVE_COMM_MODE COMM_MODE_BLE_APP  // This is now set at runtime and stored in NVS

// === NON-VOLATILE STORAGE (NVS) ===
#define NVS_NAMESPACE "nono-cfg"
#define NVS_COMM_MODE_KEY "comm_mode"

// === USB Mass Storage ===
#define USB_MSC_ENABLED false // Set to true to enable USB Mass Storage functionality


// === RTOS & TASK CONSTANTS ===
#define BLUETOOTH_TASK_STACK_SIZE 10000
#define BLUETOOTH_TASK_PRIORITY 1
#define BLUETOOTH_TASK_CORE_ID 0
#define BLUETOOTH_TASK_DELAY_MS 10
#define MUTEX_WAIT_TICKS 10
#define APP_FAILSAFE_TIMEOUT_MS 500

// === XBOX CONTROLLER CONSTANTS ===
#define GAMEPAD_NAME "Xbox Wireless Controller" // The name of your BLE Gamepad. Adjust if necessary.
#define XBOX_JOYSTICK_DEADZONE 10 // Deadzone for joystick to prevent drift (0-127)
#define XBOX_JOYSTICK_MIN -512
#define XBOX_JOYSTICK_MAX 511
#define XBOX_SPEED_INCREMENT 10

// === XBOX CONTROLLER BUTTON MAPPINGS ===
// Assign actions to controller buttons here.
//
// Available "Main" Buttons (for face buttons, shoulders, and thumbs):
// - BUTTON_A, BUTTON_B, BUTTON_X, BUTTON_Y
// - BUTTON_SHOULDER_L, BUTTON_SHOULDER_R
// - BUTTON_THUMB_L, BUTTON_THUMB_R
//
// Available "Misc" Buttons (for menu-style buttons):
// - MISC_BUTTON_BACK  (the "View" button on an Xbox controller)
// - MISC_BUTTON_HOME  (the glowing "Xbox" button)

#define XBOX_BTN_TOGGLE_HEADLIGHT     BUTTON_A
#define XBOX_BTN_TOGGLE_AVOIDANCE     BUTTON_B
#define XBOX_BTN_TOGGLE_SENTRY        BUTTON_X
#define XBOX_BTN_SPEED_UP             BUTTON_SHOULDER_R
#define XBOX_BTN_SPEED_DOWN           BUTTON_SHOULDER_L
#define XBOX_BTN_EMERGENCY_STOP       MISC_BUTTON_BACK

// --- XBOX CONTROLLER D-PAD MAPPINGS ---
// Define the actions available for the D-Pad
#define XBOX_ACTION_NONE                0
#define XBOX_ACTION_CALIBRATE_COMPASS   1

// Assign an action to each D-Pad button
#define XBOX_DPAD_UP_ACTION     XBOX_ACTION_CALIBRATE_COMPASS
#define XBOX_DPAD_DOWN_ACTION   XBOX_ACTION_NONE
#define XBOX_DPAD_LEFT_ACTION   XBOX_ACTION_NONE
#define XBOX_DPAD_RIGHT_ACTION  XBOX_ACTION_NONE


// === SAFETY & PERSISTENT MODE ===
#define ENABLE_PERSISTENT_MODE false // Set to true to make the robot continue its last action on disconnect. Use with caution!
#define LCD_PERSISTENT_MODE_ACTIVE "Persistent Mode!" // LCD message when persistent mode is active

// === HARDWARE LIBRARIES & EXTERN DECLARATIONS ===
#include <FS_MX1508.h>
#include <Esp32Servo.h>
#include <LSM303.h>
#include <VL53L1X.h>
#include <DFRobot_RGBLCD1602.h>
#include <Adafruit_NeoPixel.h>

#include "tourelle.h"

// Declare hardware objects as extern.
// The actual objects are defined in NoNo.ino
extern MX1508 motorA;
extern MX1508 motorB;
extern Servo Servodirection;
extern LSM303 *compass;
extern DFRobot_RGBLCD1602 *lcd;
extern VL53L1X *vl53;

extern Tourelle tourelle;


#endif // CONFIG_H
