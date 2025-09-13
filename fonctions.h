#ifndef FONCTIONS_H
#define FONCTIONS_H

#include <FS_MX1508.h>
#include <DFRobot_RGBLCD1602.h>
#include <Wire.h>
#include <Servo.h>
#include <LSM303.h> 
#include <UltraDistSensor.h>
#include <EEPROM.h>

// Enumération des états du robot pour une machine à états non-bloquante
enum RobotState {
  IDLE,           // En attente d'ordre
  OBSTACLE_AVOIDANCE,  // Mode évitement d'obstacles
  FOLLOW_HEADING,      // Mode navigation au cap
  MAINTAIN_HEADING,    // Arrêt avec maintien du cap (mode cap)
  MOVING_FORWARD,     // En train d'avancer (mode auto)
  MOVING_BACKWARD,    // En train de reculer (mode auto)
  MANUAL_MOVING_FORWARD,  // Avance de distance précise en mode manuel
  MANUAL_MOVING_BACKWARD, // Recule de distance précise en mode manuel
  TURNING_LEFT,       // En train de tourner à gauche (mode auto avec cap)
  TURNING_RIGHT,      // En train de tourner à droite (mode auto avec cap)
  PIR_DETECT,         // Mode détection PIR 
  MANUAL_TURNING_LEFT,  // Tourne à gauche en mode manuel (simple)
  MANUAL_TURNING_RIGHT, // Tourne à droite en mode manuel (simple)
  AVOID_MANEUVER,     // Manœuvre d'évitement (recul + virage)
  SCANNING_ENVIRONMENT, // Scan de l'environnement avec la tourelle
  SMART_TURNING,
  CALIBRATING_COMPASS // New state for compass calibration
};

// Enumération pour le mode de navigation global
enum NavigationMode {
  MANUAL_CONTROL,         // Contrôle direct des moteurs
  HEADING_CONTROL         // Contrôle par cap (navigation autonome)
};

// Variables globales pour la machine à états
extern RobotState currentState;
extern unsigned long lastActionTime;
extern bool actionStarted;

// Variable globale pour le mode de navigation
extern NavigationMode currentNavMode;

// Variables pour la manœuvre d'évitement
extern bool hasReculed;
extern bool hasTurned;

// Nouvelles constantes pour le contrôle moteur amélioré
#define VITESSE_LENTE 120
#define VITESSE_MOYENNE 180
#define VITESSE_RAPIDE 255
const float CALIBRATION_MOTEUR_B = 1.0; 
const int RAMP_STEP = 5; 
const float Kp_HEADING = 1.5;

#define LED_ROUGE A2
#define LED_JAUNE A1 
#define TRIGGER 36  //capteur bas
#define ECHO 37
#define INTERUPTPIN 39
//#define TONE A3
#define VBAT A0
//#define PINCLOCHE 12
#define PINDIRECTION 10
#define PINTOURELLE_H 12
#define PINTOURELLE_V 11
#define AIN1 2
#define AIN2 3
#define BIN1 4
#define BIN2 5
#define NUMPWM 2
#define PINBAT A0
#define PINPHARE 38
#define PIR 40

// Battery and safety constants
#define BATTERY_THRESHOLD 850
#define BATTERY_CRITICAL_THRESHOLD 700

// LED constants
#define LED_BLINK_INTERVAL 100  // milliseconds
#define LED_BLINK_COUNT 10      // number of blinks

// Turn direction constants
#define TURN_LEFT_CHOICE 1
#define TURN_RIGHT_CHOICE 3

// Servo adjustment constants
#define SERVO_ADJUSTMENT_FACTOR 0.5
#define SERVO_MAX_ADJUSTMENT 10

// Scan timing constants
#define SCAN_DELAY_MS 500
#define SERVO_STABILIZE_DELAY_MS 200

// Hardware availability flags - set to 0 to disable missing hardware
#define ENABLE_LEDS 0          // Set to 1 when LEDs are installed
#define ENABLE_TOWER 0          // Set to 1 when turret is installed
#define DEBUG_MODE 1            // Set to 0 for production (disables I2C scan)

// Constants for precise turning
#define PRECISE_TURN_MIN_SPEED 80    // Vitesse minimale pour les virages précis
#define PRECISE_TURN_MAX_SPEED 150   // Vitesse maximale pour les virages précis
#define PRECISE_TURN_SLOWDOWN_ANGLE 15.0 // Angle à partir duquel on ralentit
#define PRECISE_TURN_TOLERANCE 2.0   // Tolérance pour les virages précis

// Constants for manual distance control
#define MANUAL_DEFAULT_DISTANCE 20   // Distance par défaut en cm pour les mouvements manuels
#define MANUAL_MIN_DISTANCE 5        // Distance minimale en cm
#define MANUAL_MAX_DISTANCE 100      // Distance maximale en cm
#define MANUAL_DISTANCE_TOLERANCE 2  // Tolérance en cm pour l'arrêt

#define NEUTRE_DIRECTION 102
#define NEUTRE_TOURELLE 75
#define DMARGE 100
#define DARRET 10
#define VMINI 100
#define INITIAL_CAP 314
#define INITIAL_NCAP 200

/*
 * MX1508(uint8_t pinIN1, uint8_t pinIN2, DecayMode decayMode, NumOfPwmPins numPWM);
 * DecayMode must be FAST_DECAY or SLOW_DECAY,
 * NumOfPwmPins, either use 1 or 2 pwm. 
 * I recommend using 2 pwm pins per motor so spinning motor forward and backward gives similar response.
 * if using 1 pwm pin, make sure its pinIN1, then set pinIN2 to any digital pin. I dont recommend this setting because 
 * we need to use FAST_DECAY in one direction and SLOW_DECAY for the other direction.  
 */
 
extern MX1508 motorA; // pwm pin 17, 18, default SLOW_DECAY mode
extern MX1508 motorB; // pwm pin 17, 18, default SLOW_DECAY mode

extern UltraDistSensor sonic;
#include "tourelle.h"
extern Servo Servodirection;
extern Tourelle tourelle;
extern LSM303 compass;
extern DFRobot_RGBLCD1602 lcd;
extern String lcdText;

// Fonctions pour l'écran LCD
void setLcdText(const String& text);


// Nouvelles variables pour le contrôle moteur amélioré
extern int vitesseCible;
extern int vitesseCourante;

// Variables pour la navigation par compas
extern float capCibleRotation; // Cap cible pour les virages
const float TOLERANCE_VIRAGE = 5.0; // Tolérance en degrés pour considérer un virage terminé

// Variables pour le contrôle de distance manuel
extern int manualDistance; // Distance réglable pour les mouvements manuels
extern int manualStartDistance; // Distance de départ pour le mouvement en cours
extern bool manualMovementActive; // Indique si un mouvement de distance est en cours

// Variables pour la calibration du compas
extern float compassOffset; // Offset de calibration du compas

extern bool compassCalibrated; // Indique si le compas est calibré
extern bool compassInitialized; // Indique si le compas a été initialisé avec succès
const int CALIBRATION_SAMPLES = 50; // Nombre d'échantillons pour la calibration
extern bool compassInverted; // Correction pour compas inversé (180°)

// Variables pour la calibration 360°
extern LSM303::vector<int16_t> magMin;
extern LSM303::vector<int16_t> magMax;

// EEPROM addresses for persistent storage
const int EEPROM_MAGIC_NUMBER_ADDR = 0; // Moved to start for easier check
const int EEPROM_MAGIC_VALUE = 0x1234; // Magic number to verify data integrity
const int EEPROM_COMPASS_OFFSET_ADDR = sizeof(int); // After magic number
const int EEPROM_COMPASS_CALIBRATED_ADDR = EEPROM_COMPASS_OFFSET_ADDR + sizeof(float); // After offset
const int EEPROM_MAG_MIN_X_ADDR = EEPROM_COMPASS_CALIBRATED_ADDR + sizeof(bool);
const int EEPROM_MAG_MIN_Y_ADDR = EEPROM_MAG_MIN_X_ADDR + sizeof(int16_t);
const int EEPROM_MAG_MIN_Z_ADDR = EEPROM_MAG_MIN_Y_ADDR + sizeof(int16_t);
const int EEPROM_MAG_MAX_X_ADDR = EEPROM_MAG_MIN_Z_ADDR + sizeof(int16_t);
const int EEPROM_MAG_MAX_Y_ADDR = EEPROM_MAG_MAX_X_ADDR + sizeof(int16_t);
const int EEPROM_MAG_MAX_Z_ADDR = EEPROM_MAG_MAX_Y_ADDR + sizeof(int16_t);

// Error codes for visual reporting
const int ERROR_COMPASS_INIT = 1;      // 1 blink = Compass initialization failed
const int ERROR_COMPASS_READ = 2;       // 2 blinks = Compass read failed
const int ERROR_BATTERY_LOW = 3;        // 3 blinks = Battery low
const int ERROR_SENSOR_ULTRASONIC = 4;  // 4 blinks = Ultrasonic sensor error
const int ERROR_MOTOR_STUCK = 5;        // 5 blinks = Motor stuck

extern int dusm; // Ultrasonic distance measurement
extern byte neutredirection;
extern byte neutretourelle;
extern byte dmarge, darret;
extern byte vmini;            //distance de ralentissement
extern int Ncap, cap;

// Fonctions existantes
void Mcap(int j);
float calculateHeading(LSM303 &compass);
void PhareAllume();
void Phareeteint();
void Batterie();
void Arret();
void Terminal();

// Fonctions motrices non-bloquantes
void changeState(RobotState newState);
float calculateHeadingError(float target, float current);
float calculateTargetHeading(float currentHeading, float turnAngle);

// Fonctions de débogage et calibration du compas
void calibrateCompass();
void debugCompass();
void displayCompassInfo();
float getCalibratedHeading(LSM303 &compass);

// Fonctions de gestion EEPROM pour la calibration
void saveCompassCalibration();
void loadCompassCalibration();
bool isEEPROMDataValid();

// Fonctions de virage améliorées
void executePreciseTurn(float targetAngle);
float calculateTurnSpeed(float error);

// Fonctions de contrôle de distance manuel
void executeManualDistanceMove(bool forward);
bool checkManualDistanceComplete();
void setManualDistance(int distance);

// Nouvelle fonction de contrôle moteur centralisée
void updateMotorControl();

#include "balises.h"
#include "fonctions_motrices.h"
#include "support.h"

#endif // FONCTIONS_H