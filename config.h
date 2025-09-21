#ifndef CONFIG_H
#define CONFIG_H

// === DEBUG MODE ===
#define DEBUG_MODE true // Master switch for serial debug output

// === PIN DEFINITIONS ===
// Motor A (Droite)
#define AIN1 2
#define AIN2 3
// Motor B (Gauche)
#define BIN1 4
#define BIN2 5
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
#define DMARGE 30
#define DARRET 15
#define VMINI 80
#define INITIAL_CAP 0
#define INITIAL_NCAP 0
#define VITESSE_LENTE 100
#define VITESSE_MOYENNE 150
#define VITESSE_RAPIDE 200
#define VITESSE_ROTATION 120
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

// === SCANNING CONSTANTS ===
#define SCAN_H_START_ANGLE 30
#define SCAN_H_END_ANGLE 150
#define SCAN_H_STEP 10
#define SCAN_DELAY_MS 100 // Delay between each scan step

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
  SCANNING_ENVIRONMENT,
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