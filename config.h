#ifndef CONFIG_H
#define CONFIG_H

// === DEBUG MODE ===
#define DEBUG_MODE true // Master switch for serial debug output

// === PIN DEFINITIONS ===
// Motor A (Droite)
#define AIN1 3
#define AIN2 2
// Motor B (Gauche)
#define BIN1 5
#define BIN2 4
// Ultrasonic Sensor
#define TRIGGER 36
#define ECHO 37
// Servos
#define PINDIRECTION 10
#define PINTOURELLE_H 8
#define PINTOURELLE_V 9
// LEDs
#define LED_ROUGE 22
#define LED_JAUNE 24
// Headlight
#define PIN_PHARE 38
// Other
#define VBAT A0 // Broche pour la mesure de la tension de la batterie
#define INTERUPTPIN 39 // Broche pour le bouton d'arrêt d'urgence
#define PIR 40 // Broche pour le capteur de mouvement PIR
#define SD_CS_PIN 26 // Chip Select pin for SD card module
#define BUZZER_PIN 6 // Pin for passive buzzer

#define PWM_MAX 255
#define ENABLE_LEDS false // Mettre à true lorsque les LEDs sont installées

#define NEUTRE_DIRECTION 90
#define NEUTRE_TOURELLE 90
#define ULTRASONIC_OBSTACLE_THRESHOLD_CM 10 // Ultrasonic obstacle threshold in cm
#define LASER_OBSTACLE_THRESHOLD_CM 20   // Laser obstacle threshold in cm
#define VMINI 150
#define INITIAL_CAP 0
#define INITIAL_NCAP 0
#define VITESSE_LENTE 150
#define VITESSE_MOYENNE 200
#define VITESSE_RAPIDE 250
#define VITESSE_ROTATION 200
#define RAMP_STEP 5
#define ACCELERATION_STEP 10
#define DECELERATION_STEP 10
#define HEADING_TOLERANCE 5.0
#define TOLERANCE_VIRAGE 2.0
#define Kp_ROTATION 5.0
#define Kp_HEADING 1.5
#define SERVO_ADJUSTMENT_FACTOR 2.0
#define SERVO_MAX_ADJUSTMENT 20
#define CALIBRATION_MOTEUR_B 1.0
#define SCAN_CENTER_ANGLE 90
#define COMPASS_IS_INVERTED true // Mettre à true si la boussole est montée à l'envers


#define SENTRY_ALARM_DURATION_MS 5000
#define SENTRY_ALARM_BLINK_DIVISOR 2
#define TURNING_TIMEOUT_MS 5000
#define AVOID_BACKUP_DURATION_MS 1000
#define SENTRY_FLASH_INTERVAL_MS 250
#define TURRET_MOVE_TIME_MS 200


#define SCAN_DISTANCE_ARRAY_SIZE 181
#define SCAN_H_START_ANGLE 10
#define SCAN_H_END_ANGLE 170
#define SCAN_H_STEP 5
#define SCAN_DELAY_MS 100 // Delay between each scan step
#define QUICK_SCAN_LEFT_ANGLE 70
#define QUICK_SCAN_RIGHT_ANGLE 110


#define MM_TO_CM_DIVISOR 10
#define VL53L1X_TIMING_BUDGET_US 50000
#define VL53L1X_INTER_MEASUREMENT_PERIOD_MS 50
#define MAX_ULTRASONIC_DISTANCE 400 // Max valid distance in cm (e.g., 400cm = 4m)
#define ULTRASONIC_PING_INTERVAL_MS 60 // Minimum time between pings
#define ULTRASONIC_PULSE_TIMEOUT_US 25000 // Max wait time for echo in µs (25ms ~ 4.3m)
#define ULTRASONIC_TRIGGER_PULSE_LOW_US 2
#define ULTRASONIC_TRIGGER_PULSE_HIGH_US 10
#define ULTRASONIC_DURATION_TO_CM_DIVISOR 58
#define ULTRASONIC_ERROR_VALUE -1


// === COMPASS CONSTANTS ===
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
#define ANGLE_TETE_BASSE 120 // "Sad" head angle


#define SERIAL_BAUD_RATE 115200
#define LCD_I2C_ADDR 0x60
#define LCD_ROWS 2

#define LCD_LINE_LENGTH 16
#define JSON_DOC_SIZE 200
#define CMD_BUFFER_SIZE 64
#define MAX_LCD_TEXT_LENGTH 64
#define SCROLL_DELAY_MS     2000 // The display time of each page (in ms)
#define LCD_IDLE_TIMEOUT_MS 5000 // 5 seconds of inactivity before jokes start
#define LCD_JOKE_INTERVAL_MS 5000 // Change joke every 5 seconds if still idle

// === INITIALIZATION ===
#define INITIAL_AUTONOMOUS_DELAY_MS 10000 // Delay before initial autonomous action at startup
#define MAX_CONSECUTIVE_AVOID_MANEUVERS 3

// === INITIALIZATION ===


// === LCD MESSAGES ===
#define LCD_STARTUP_MESSAGE_1 "Je suis Nono"
#define LCD_STARTUP_MESSAGE_2 "Paré à exploser...explorer pardon"

#define SCAN_V_START_ANGLE NEUTRE_TOURELLE
#define SCAN_V_END_ANGLE NEUTRE_TOURELLE // For now, only scan at neutral vertical angle
#define SCAN_V_STEP 1 // Not used for now, but good to have

// === HEAD ANIMATIONS ===
#define HEAD_SHAKE_NO_ANGLE_EXTREME_LEFT 135
#define HEAD_SHAKE_NO_ANGLE_EXTREME_RIGHT 45
#define HEAD_SHAKE_NO_CYCLE_DURATION_MS 200
#define HEAD_NOD_YES_ANGLE_UP 70
#define HEAD_NOD_YES_ANGLE_DOWN 110
#define HEAD_NOD_YES_CYCLE_DURATION_MS 200

// === CLIFF DETECTION ===
#define ANGLE_SOL 100 // Angle to look down at the ground (to be calibrated)
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
  SCANNING,
  SMART_AVOIDANCE,
  SENTRY_MODE,
  SENTRY_ALARM,
  PLAYING_MUSIC,
  CHECKING_GROUND,
  CLIFF_DETECTED,
  ANIMATING_HEAD
};

enum NavigationMode {
  MANUAL_CONTROL,
  AUTONOMOUS_CONTROL
};

#endif // CONFIG_H
