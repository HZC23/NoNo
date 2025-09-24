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

// === HARDWARE & BEHAVIOR CONSTANTS ===
#define ENABLE_LEDS false // Mettre à true lorsque les LEDs sont installées
#define ENABLE_TOWER true // Mettre à true lorsque la tourelle est installée
#define NEUTRE_DIRECTION 90
#define NEUTRE_TOURELLE 90
#define DMARGE 20 // Marge de sécurité en cm pour la détection d'obstacles
#define DARRET 15 // Distance d'arrêt d'urgence en cm
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

// === SCANNING CONSTANTS ===
#define SCAN_H_START_ANGLE 10
#define SCAN_H_END_ANGLE 170
#define SCAN_H_STEP 10
#define SCAN_DELAY_MS 200 // Delay between each scan step

// === INITIALIZATION ===
#define INITIAL_AUTONOMOUS_DELAY_MS 10000 // Delay before initial autonomous action at startup
#define MAX_CONSECUTIVE_AVOID_MANEUVERS 3

// === INITIALIZATION ===


// === LCD MESSAGES ===
#define LCD_STARTUP_MESSAGE_1 "Je suis NONO"
#define LCD_STARTUP_MESSAGE_2 "Pret."

#define SCAN_V_START_ANGLE NEUTRE_TOURELLE
#define SCAN_V_END_ANGLE NEUTRE_TOURELLE // For now, only scan at neutral vertical angle
#define SCAN_V_STEP 1 // Not used for now, but good to have

// === SENSOR CONSTANTS ===
#define MAX_ULTRASONIC_DISTANCE 400 // Max valid distance in cm (e.g., 400cm = 4m)

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
  FOLLOW_HEADING,
  MAINTAIN_HEADING,
  AVOID_MANEUVER,
  BACKING_UP_OBSTACLE, // New state for backing up from obstacle
  SCANNING_FOR_PATH,   // New state for scanning for a new path
  TURNING_TO_PATH,     // New state for turning to the new path
  SMART_TURNING,
  CALIBRATING_COMPASS,
  SCANNING,
  SMART_AVOIDANCE,
  SENTRY_MODE,
  SENTRY_ALARM
};

enum NavigationMode {
  MANUAL_CONTROL,
  AUTONOMOUS_CONTROL
};

#endif // CONFIG_H