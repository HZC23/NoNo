// --- INCLUDES ---
#include <Arduino.h>
#include <Wire.h>
#include <VL53L1X.h>
#include <ArduinoJson.h> // Required for telemetry
#include "config.h"      // Central configuration for pins and constants
#include "state.h"       // Global state and hardware objects
#include "compass.h"     // Compass logic
#include "display.h"     // LCD display functions
#include "terminal.h"    // Serial command processing
#include "sensor_task.h" // Ultrasonic sensor task
#include "fonctions_motrices.h" // Motor control state machine
#include "balises.h"     // LED indicators
#include "support.h"     // Misc support functions
#include "tourelle.h"    // Turret class
#include "telemetry.h"   // For sending JSON data
// #include "sd_utils.h"    // For SD card utilities

// --- HARDWARE OBJECTS DEFINITION ---
MX1508 motorA(AIN1, AIN2);
MX1508 motorB(BIN1, BIN2);
Servo Servodirection; // Ackermann steering
Tourelle tourelle(PINTOURELLE_H, PINTOURELLE_V);
LSM303 compass;
DFRobot_RGBLCD1602 lcd(LCD_I2C_ADDR, LCD_LINE_LENGTH, LCD_ROWS);
VL53L1X vl53;

// --- GLOBAL STATE OBJECT ---
Robot robot;
unsigned long startTime;

// --- FORWARD DECLARATIONS ---
void handleScanning(Robot& robot);

// --- SETUP ---
void setup() {
    digitalWrite(PIN_PHARE, HIGH); // Turn on headlight at the beginning of setup
    Serial.begin(SERIAL_BAUD_RATE);
    Wire.begin();
    lcd.init();
    randomSeed(analogRead(0)); // Initialize random number generator
    if (DEBUG_MODE) Serial.println("--- NONO STARTUP ---");
    startTime = millis(); // Initialize startupTime

    if (DEBUG_MODE) Serial.println("Initializing VL53L1X...");
    vl53.setBus(&Wire);
    if (!vl53.init()) {
        Serial.println(F("Failed to boot VL53L1X"));
        while(1);
    }
    if (DEBUG_MODE) Serial.println("VL53L1X Initialized.");
    vl53.setMeasurementTimingBudget(VL53L1X_TIMING_BUDGET_US);
    vl53.startContinuous(VL53L1X_INTER_MEASUREMENT_PERIOD_MS);

    setLcdText(robot, LCD_STARTUP_MESSAGE_1); // Keep the first startup message for a moment

    // Init hardware
    pinMode(PIR, INPUT);
    pinMode(VBAT, INPUT);
    pinMode(INTERUPTPIN, INPUT_PULLUP);
    pinMode(PIN_PHARE, OUTPUT); // Initialize headlight pin
    
    #if ENABLE_LEDS
      pinMode(LED_ROUGE, OUTPUT);
      digitalWrite(LED_ROUGE, LOW);
      pinMode(LED_JAUNE, OUTPUT);
      digitalWrite(LED_JAUNE, LOW);
    #endif

    // Init Sensors & Servos
    sensor_init();
    compass_init(robot);
    robot.compassInverted = COMPASS_IS_INVERTED; // Apply inversion setting from config

    // Initialize configurable parameters
    robot.speedAvg = VITESSE_MOYENNE;
    robot.speedSlow = VITESSE_LENTE;

    Servodirection.attach(PINDIRECTION, 70, 105); // Ackermann steering
    Servodirection.write(NEUTRE_DIRECTION);      // Ackermann steering
    #if ENABLE_TOWER
      tourelle.attach();
      tourelle.write(SCAN_CENTER_ANGLE, NEUTRE_TOURELLE);
    #endif

    // Initialize SD card
    // if (!setupSDCard()) {
    //     // Handle SD card error, perhaps loop indefinitely or log
    //     setLcdText(robot, "SD Card Error!");
    //     while (true); 
    // }

    if (DEBUG_MODE) Serial.println("--- SETUP COMPLETE ---");
    setLcdText(robot, LCD_STARTUP_MESSAGE_2);
    digitalWrite(PIN_PHARE, LOW); // Turn off headlight at the end of setup
}


// --- MAIN LOOP ---

void loop() {

  // Check battery level first

  if (readBatteryPercentage() == 0) {

    setLcdText(robot, "Pas de batterie");

    // Optional: Stop the robot completely if battery is dead.

    if(robot.currentState != IDLE) {

      changeState(robot, IDLE); // This will also stop the motors

    }
    delay(1000); // Wait a second before checking again

  }



  // Process incoming commands

  Terminal(robot); 

 

  // Transition to OBSTACLE_AVOIDANCE only once at startup
  if (!robot.initialActionTaken && robot.currentState == IDLE && millis() - startTime >= INITIAL_AUTONOMOUS_DELAY_MS) {
      changeState(robot, OBSTACLE_AVOIDANCE);
      robot.currentNavMode = AUTONOMOUS_CONTROL; // Set navigation mode to autonomous
      robot.initialActionTaken = true;
  }

  // Update sensors
  sensor_update_task(robot);
  robot.cap = getCalibratedHeading(robot);
  if (vl53.dataReady()) {
    robot.distanceLaser = vl53.readRangeContinuousMillimeters() / MM_TO_CM_DIVISOR;
  }

  // Run state machines
  switch (robot.currentState) {
    case CALIBRATING_COMPASS:
      calibrateCompass(robot);
      break;

    case SCANNING: // Unified scanning state
      handleScanning(robot);
      break;

    case PLAYING_MUSIC:
      // Play music from SD card, then return to IDLE
      setLcdText(robot, "Playing Music... (SD disabled)");
      // playMusicFromSD(robot.musicFileName.c_str(), BUZZER_PIN); // SD card disabled
      setLcdText(robot, "Music Finished");
      changeState(robot, IDLE); // Return to IDLE after playing music
      break;

    default:
      updateMotorControl(robot);
      break;
  }

  // Update LCD display with current status or jokes if idle
  displayJokesIfIdle(robot);
  updateLcdDisplay(robot);

  // Send telemetry back to the app periodically
  if (millis() - robot.lastReportTime > robot.reportInterval) {
    robot.lastReportTime = millis();
    sendTelemetry(robot);
  }
}

// --- STATE HANDLERS ---

void handleScanning(Robot& robot) {
  if (!robot.actionStarted) {
    robot.currentScanAngleH = SCAN_H_START_ANGLE;
    tourelle.write(robot.currentScanAngleH, NEUTRE_TOURELLE); // Start scan at 90 deg vertical
    robot.lastScanTime = millis();
    robot.actionStarted = true;
    if (DEBUG_MODE) Serial.println(F("Starting unified scan."));
  }

  if (millis() - robot.lastScanTime >= SCAN_DELAY_MS) {
    robot.lastScanTime = millis();
    
    // Report current angle and distance
    if (DEBUG_MODE) {
      Serial.print(F("Scan: Angle="));
      Serial.print(robot.currentScanAngleH);
      Serial.print(F(", Dist="));
      Serial.println(robot.distanceLaser);
    }

    robot.currentScanAngleH += SCAN_H_STEP;
    if (robot.currentScanAngleH > SCAN_H_END_ANGLE) {
      tourelle.write(SCAN_CENTER_ANGLE, NEUTRE_TOURELLE); // Center turret when done
      changeState(robot, IDLE); // Scan complete
      if (DEBUG_MODE) Serial.println(F("Scan complete."));
    } else {
      tourelle.write(robot.currentScanAngleH, NEUTRE_TOURELLE);
    }
  }
}

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
#define ENABLE_TOWER true // Mettre à true lorsque la tourelle est installée
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
#define SCAN_H_STEP 10
#define SCAN_DELAY_MS 200 // Delay between each scan step
#define QUICK_SCAN_LEFT_ANGLE 70
#define QUICK_SCAN_RIGHT_ANGLE 110


#define MM_TO_CM_DIVISOR 10
#define VL53L1X_TIMING_BUDGET_US 50000
#define VL53L1X_INTER_MEASUREMENT_PERIOD_MS 50
#define MAX_ULTRASONIC_DISTANCE 400 // Max valid distance in cm (e.g., 400cm = 4m)
#define ULTRASONIC_PING_INTERVAL_MS 60 // Minimum time between pings
#define ULTRASONIC_PULSE_TIMEOUT_US 30000 // Max wait time for echo in µs (30ms ~ 5m)
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


#define SERIAL_BAUD_RATE 115200
#define LCD_I2C_ADDR 0x60
#define LCD_ROWS 2

#define LCD_LINE_LENGTH 16
#define JSON_DOC_SIZE 200
#define CMD_BUFFER_SIZE 64
#define MAX_LCD_TEXT_LENGTH 32
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
  PLAYING_MUSIC
};

enum NavigationMode {
  MANUAL_CONTROL,
  AUTONOMOUS_CONTROL
};

#endif // CONFIG_H
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

// Structure to hold the robot's state
struct Robot {
    // State Machine
    RobotState currentState = IDLE;
    ObstacleAvoidanceState obstacleAvoidanceState = AVOID_START;
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

    // Manual Override
    int manualDistance = 0;
    int manualStartDistance = 0;
    bool manualMovementActive = false;

    // Sensors
    int dusm = 0; // Distance UltraSon Mesuree
    int distanceLaser = 0;
    bool obstacleDetectedByLaser = false;

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
    String lcdText = "";
    unsigned long lastLcdUpdateTime = 0;
    unsigned long lastJokeDisplayTime = 0;
    String musicFileName = ""; // To store the name of the music file to play

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

#ifndef TOURELLE_H
#define TOURELLE_H

#include <Servo.h>
#include "config.h" // Include for constants if needed

class Tourelle {
private:
    Servo servoH;
    Servo servoV;
    int servoH_pin;
    int servoV_pin;

public:
    // Constructor with initializer list for better practice
    Tourelle(int pinH, int pinV) : servoH_pin(pinH), servoV_pin(pinV) {}

    // Attach servos to pins
    void attach() {
      servoH.attach(servoH_pin);
      servoV.attach(servoV_pin);
    }

    void detach() {
      servoH.detach();
      servoV.detach();
    }

    // Write angles to servos (restored and implemented)
    void write(int angleH, int angleV) {
      servoH.write(angleH);
      servoV.write(angleV);
    }

    // Get current angles (can be improved later)
    int getAngleHorizontal() {
        return servoH.read();
    }

    int getAngleVertical() {
        return servoV.read();
    }
};

#endif // TOURELLE_H
#ifndef SUPPORT_H
#define SUPPORT_H

#include <Arduino.h>
#include "config.h"

// Prototypes
void PhareAllume();
void PhareEteint();
int readBatteryPercentage();
bool isValidNumericInput(const String& input, int minVal, int maxVal);

// Implementations
inline void PhareAllume() {
  digitalWrite(PIN_PHARE, HIGH);
}

inline void PhareEteint() {
  digitalWrite(PIN_PHARE, LOW);
}

inline int readBatteryPercentage() {
  // Based on a voltage divider with R1=6.8k and R2=10k
  // V_pin = V_battery * (R2 / (R1 + R2))
  // V_battery = V_pin * ((R1 + R2) / R2) = V_pin * 1.68
  int sensorValue = analogRead(VBAT);
  float pinVoltage = sensorValue * (5.0 / 1023.0);
  float batteryVoltage = pinVoltage * 1.68;

  // Convert voltage to percentage based on selected battery type
  float maxVoltage, minVoltage;

#if SELECTED_BATTERY_TYPE == BATTERY_TYPE_LIPO
  maxVoltage = LIPO_MAX_VOLTAGE;
  minVoltage = LIPO_MIN_VOLTAGE;
#elif SELECTED_BATTERY_TYPE == BATTERY_TYPE_NIMH
  maxVoltage = NIMH_MAX_VOLTAGE;
  minVoltage = NIMH_MIN_VOLTAGE;
#else
  // Default to LiPo if no type is selected
  maxVoltage = LIPO_MAX_VOLTAGE;
  minVoltage = LIPO_MIN_VOLTAGE;
#endif

  float percentage = ((batteryVoltage - minVoltage) / (maxVoltage - minVoltage)) * 100.0;
  
  // Constrain the value between 0 and 100
  return constrain((int)percentage, 0, 100);
}

inline bool isValidNumericInput(const String& input, int minVal, int maxVal) {
  for (unsigned int i = 0; i < input.length(); i++) {
    if (!isDigit(input.charAt(i))) {
      return false;
    }
  }
  long val = input.toInt();
  return (val >= minVal && val <= maxVal);
}

#endif // SUPPORT_H

#ifndef BALISES_H
#define BALISES_H

#include "config.h"

// Prototypes
void balise_jaune();
void balise_rouge();
void balises_off();

// Implementations
inline void balise_jaune() {
  #if ENABLE_LEDS
    digitalWrite(LED_ROUGE, LOW);
    digitalWrite(LED_JAUNE, HIGH);
  #endif
}

inline void balise_rouge() {
  #if ENABLE_LEDS
    digitalWrite(LED_JAUNE, LOW);
    digitalWrite(LED_ROUGE, HIGH);
  #endif
}

inline void balises_off() {
  #if ENABLE_LEDS
    digitalWrite(LED_ROUGE, LOW);
    digitalWrite(LED_JAUNE, LOW);
  #endif
}

#endif // BALISES_H
#ifndef SD_UTILS_H
#define SD_UTILS_H

#include <Arduino.h>
#include <SD.h> // Include the SD library

// Forward declaration for joke reading function
String getRandomJokeFromSD(const char* filename);
bool setupSDCard();

// Forward declaration for music playback function
void playMusicFromSD(const char* filename, int buzzerPin);
void listMusicFiles(void (*callback)(const char* filename));

#endif // SD_UTILS_H

#ifndef DISPLAY_H
#define DISPLAY_H

#include <Arduino.h>
#include "state.h" // Need access to Robot struct and RobotState enum
#include "config.h" // For LCD_LINE_LENGTH and other constants
#include "DFRobot_RGBLCD1602.h" // For lcd object
#include "sd_utils.h" // For SD card utility functions

// Declare extern if lcd object is defined in NoNo.ino
extern DFRobot_RGBLCD1602 lcd;



inline void setLcdText(Robot& robot, const String& text) {
    if (text == robot.lcdText) return; // Avoid unnecessary screen redraws
    lcd.clear();

    if (text.length() <= LCD_LINE_LENGTH) {
        lcd.setCursor(0, 0);
        lcd.print(text);
        lcd.setCursor(0, 1);
        char spaces[LCD_LINE_LENGTH + 1];
        memset(spaces, ' ', LCD_LINE_LENGTH);
        spaces[LCD_LINE_LENGTH] = '\0';
        lcd.print(spaces);
    } else { // text.length() > LCD_LINE_LENGTH and <= MAX_LCD_TEXT_LENGTH
        lcd.setCursor(0, 0);
        lcd.print(text.substring(0, LCD_LINE_LENGTH));
        lcd.setCursor(0, 1);
        lcd.print(text.substring(LCD_LINE_LENGTH));
    }
    robot.lcdText = text;
    robot.lastLcdUpdateTime = millis(); // Update timestamp on actual display change
}

inline void displayRandomJoke(Robot& robot) {
    String joke = getRandomJokeFromSD("jokes.txt"); // Assuming jokes.txt is the file
    setLcdText(robot, joke);
    robot.lastJokeDisplayTime = millis();
}

void displayJokesIfIdle(Robot& robot) {
    unsigned long currentTime = millis();
    if (currentTime - robot.lastLcdUpdateTime >= LCD_IDLE_TIMEOUT_MS) {
        // LCD has been idle, now check if it's time to change the joke
        if (currentTime - robot.lastJokeDisplayTime >= LCD_JOKE_INTERVAL_MS || robot.lastJokeDisplayTime == 0) { // If it's the first time displaying a joke, lastJokeDisplayTime will be 0
            displayRandomJoke(robot);
        }
    }
}

inline void updateLcdDisplay(Robot& robot) {
    String displayString = ""; // This will hold both lines
    char buffer[LCD_LINE_LENGTH + 1]; // Buffer for formatted strings

    switch (robot.currentState) {
        case IDLE:
            displayString += "IDLE            "; // 16 chars
            displayString += "Waiting...      ";
            break;
        case MOVING_FORWARD:
            snprintf(buffer, sizeof(buffer), "FORWARD         ");
            displayString += buffer;
            snprintf(buffer, sizeof(buffer), "CAP:%-11d", robot.cap); // 16 chars
            displayString += buffer;
            break;
        case MOVING_BACKWARD:
            snprintf(buffer, sizeof(buffer), "BACKWARD        ");
            displayString += buffer;
            snprintf(buffer, sizeof(buffer), "CAP:%-11d", robot.cap);
            displayString += buffer;
            break;
        case TURNING_LEFT:
            snprintf(buffer, sizeof(buffer), "TURN LEFT       ");
            displayString += buffer;
            snprintf(buffer, sizeof(buffer), "CAP:%-11d", robot.cap);
            displayString += buffer;
            break;
        case TURNING_RIGHT:
            snprintf(buffer, sizeof(buffer), "TURN RIGHT      ");
            displayString += buffer;
            snprintf(buffer, sizeof(buffer), "CAP:%-11d", robot.cap);
            displayString += buffer;
            break;
        case MANUAL_FORWARD:
            snprintf(buffer, sizeof(buffer), "MANUAL FWD      ");
            displayString += buffer;
            snprintf(buffer, sizeof(buffer), "CAP:%-11d", robot.cap);
            displayString += buffer;
            break;
        case MANUAL_BACKWARD:
            snprintf(buffer, sizeof(buffer), "MANUAL BACK     ");
            displayString += buffer;
            snprintf(buffer, sizeof(buffer), "CAP:%-11d", robot.cap);
            displayString += buffer;
            break;
        case MANUAL_TURNING_LEFT:
            snprintf(buffer, sizeof(buffer), "MANUAL TURN L   ");
            displayString += buffer;
            snprintf(buffer, sizeof(buffer), "CAP:%-11d", robot.cap);
            displayString += buffer;
            break;
        case MANUAL_TURNING_RIGHT:
            snprintf(buffer, sizeof(buffer), "MANUAL TURN R   ");
            displayString += buffer;
            snprintf(buffer, sizeof(buffer), "CAP:%-11d", robot.cap);
            displayString += buffer;
            break;
        case OBSTACLE_AVOIDANCE:
        case SMART_AVOIDANCE:
            snprintf(buffer, sizeof(buffer), "AVOIDING        ");
            displayString += buffer;
            snprintf(buffer, sizeof(buffer), "US:%-4d L:%-4d", robot.dusm, robot.distanceLaser); // 16 chars
            displayString += buffer;
            break;
        case CALIBRATING_COMPASS:
            displayString += "CALIBRATING     ";
            displayString += "COMPASS...      ";
            break;
        case SENTRY_MODE:
            displayString += "SENTRY MODE     ";
            displayString += "Scanning...     ";
            break;
        case SENTRY_ALARM:
            displayString += "SENTRY ALARM    ";
            displayString += "INTRUDER!!!     ";
            break;
        case FOLLOW_HEADING:
            snprintf(buffer, sizeof(buffer), "FOLLOW HDG      ");
            displayString += buffer;
            snprintf(buffer, sizeof(buffer), "TRG:%-4d CUR:%-4d", (int)robot.capCibleRotation, robot.cap);
            displayString += buffer;
            break;
        case MAINTAIN_HEADING:
            snprintf(buffer, sizeof(buffer), "MAINTAIN HDG    ");
            displayString += buffer;
            snprintf(buffer, sizeof(buffer), "TRG:%-4d CUR:%-4d", (int)robot.capCibleRotation, robot.cap);
            displayString += buffer;
            break;
        case SCANNING:
            snprintf(buffer, sizeof(buffer), "SCANNING        ");
            displayString += buffer;
            snprintf(buffer, sizeof(buffer), "ANG:%-4d DIST:%-4d", robot.currentScanAngleH, robot.distanceLaser);
            displayString += buffer;
            break;
        default:
            snprintf(buffer, sizeof(buffer), "STATE: %-8d", robot.currentState);
            displayString += buffer;
            displayString += "UNKNOWN         ";
            break;
    }
    setLcdText(robot, displayString);
}

#endif // DISPLAY_H
#ifndef COMPASS_H
#define COMPASS_H

#include <Arduino.h>
#include <LSM303.h>
#include <EEPROM.h>
#include "state.h"
#include "display.h" // For setLcdText

// EEPROM addresses for calibration data
#define EEPROM_MAGIC_NUMBER_ADDR 0
#define EEPROM_COMPASS_OFFSET_ADDR 4
#define EEPROM_COMPASS_CALIBRATED_ADDR 8
#define EEPROM_MAG_MIN_X_ADDR 12
#define EEPROM_MAG_MIN_Y_ADDR 16
#define EEPROM_MAG_MIN_Z_ADDR 20
#define EEPROM_MAG_MAX_X_ADDR 24
#define EEPROM_MAG_MAX_Y_ADDR 28
#define EEPROM_MAG_MAX_Z_ADDR 32
#define EEPROM_MAGIC_VALUE 12345

// Function Prototypes
void compass_init(Robot& robot);
float getCalibratedHeading(Robot& robot);
void calibrateCompass(Robot& robot);
void saveCompassCalibration(const Robot& robot);
void loadCompassCalibration(Robot& robot);
bool isEEPROMDataValid();
float calculateHeading(float y, float x); // Overloaded
float calculateHeading(const LSM303& compass);
void displayCompassInfo(Robot& robot);


// --- IMPLEMENTATIONS ---

// Overloaded function to calculate heading from y and x components
inline float calculateHeading(float y, float x) {
  float heading = (atan2(y, x) * 180) / PI;
  if (heading < 0) {
    heading += 360;
  }
  return heading;
}

inline float calculateHeading(const LSM303& compass) {
  return calculateHeading(compass.m.y, compass.m.x);
}

inline void compass_init(Robot& robot) {
    if (DEBUG_MODE) Serial.println("Initialisation du compas...");
    if (!compass.init()) {
        if (DEBUG_MODE) Serial.println("ERREUR: Impossible d'initialiser le compas!");
        setLcdText(robot, "ERREUR COMPAS");
        robot.compassInitialized = false;
    } else {
        if (DEBUG_MODE) Serial.println("Compas initialise avec succes!");
        robot.compassInitialized = true;
        compass.enableDefault();
        loadCompassCalibration(robot);
    }
}

inline float getCalibratedHeading(Robot& robot) {
    if (!robot.compassInitialized) return 0.0;
    
    compass.read();
    if (compass.m.x == 0 && compass.m.y == 0 && compass.m.z == 0) {
        if (DEBUG_MODE) Serial.println("ATTENTION: Valeurs compas nulles dans getCalibratedHeading()");
        return 0.0;
    }

    float heading;
    if (robot.compassCalibrated) {
        // Correct for hard-iron distortion
        float corrected_x = compass.m.x - (robot.magMin.x + robot.magMax.x) / 2.0;
        float corrected_y = compass.m.y - (robot.magMin.y + robot.magMax.y) / 2.0;

        // Correct for soft-iron distortion
        float avg_delta_x = (robot.magMax.x - robot.magMin.x) / 2.0;
        float avg_delta_y = (robot.magMax.y - robot.magMin.y) / 2.0;
        
        if (avg_delta_x == 0 || avg_delta_y == 0) {
            heading = calculateHeading(compass);
        } else {
            float avg_delta = (avg_delta_x + avg_delta_y) / 2.0;
            float scaled_x = corrected_x * (avg_delta / avg_delta_x);
            float scaled_y = corrected_y * (avg_delta / avg_delta_y);
            heading = calculateHeading(scaled_y, scaled_x);
        }
    } else {
        heading = calculateHeading(compass);
    }

    // Apply inversion and fine-tuning offset
    if (robot.compassInverted) {
        heading += 180.0;
    }
    heading += robot.compassOffset;

    // Normalize heading to 0-359.9 degrees
    while (heading < 0.0) heading += 360.0;
    while (heading >= 360.0) heading -= 360.0;
    
    return heading;
}

inline void calibrateCompass(Robot& robot) {
    static unsigned long startTime = 0;
    static bool calibrationStarted = false;

    if (!calibrationStarted) {
        Serial.println("=== CALIBRATION DU COMPAS (360 DEGRES) ===");
        robot.magMin = {COMPASS_MAX_INT16, COMPASS_MAX_INT16, COMPASS_MAX_INT16};
        robot.magMax = {COMPASS_MIN_INT16, COMPASS_MIN_INT16, COMPASS_MIN_INT16};
        startTime = millis();
        calibrationStarted = true;
        setLcdText(robot, "Calib 360...");
    }

    if (millis() - startTime < COMPASS_CALIBRATION_DURATION_MS) {
        compass.read();
        robot.magMin.x = min(robot.magMin.x, compass.m.x);
        robot.magMin.y = min(robot.magMin.y, compass.m.y);
        robot.magMin.z = min(robot.magMin.z, compass.m.z);
        robot.magMax.x = max(robot.magMax.x, compass.m.x);
        robot.magMax.y = max(robot.magMax.y, compass.m.y);
        robot.magMax.z = max(robot.magMax.z, compass.m.z);
    } else {
        Serial.println("=== CALIBRATION TERMINEE ===");
        if (robot.magMin.x < COMPASS_CALIBRATION_VALIDATION_THRESHOLD && robot.magMax.x > -COMPASS_CALIBRATION_VALIDATION_THRESHOLD) {
            robot.compassCalibrated = true;
            Serial.println("✅ Calibration REUSSIE.");
            setLcdText(robot, "Calib. OK");
        } else {
            robot.compassCalibrated = false;
            Serial.println("⚠️ Calibration DEFAILLANTE.");
            setLcdText(robot, "Calib. ECHEC");
        }
        saveCompassCalibration(robot);
        calibrationStarted = false;
        robot.currentState = IDLE; // Return to IDLE state
    }
}

inline void saveCompassCalibration(const Robot& robot) {
    if (DEBUG_MODE) Serial.println("Sauvegarde de la calibration en EEPROM...");
    EEPROM.put(EEPROM_MAGIC_NUMBER_ADDR, EEPROM_MAGIC_VALUE);
    EEPROM.put(EEPROM_COMPASS_OFFSET_ADDR, robot.compassOffset);
    EEPROM.put(EEPROM_COMPASS_CALIBRATED_ADDR, robot.compassCalibrated);
    EEPROM.put(EEPROM_MAG_MIN_X_ADDR, robot.magMin.x);
    EEPROM.put(EEPROM_MAG_MIN_Y_ADDR, robot.magMin.y);
    EEPROM.put(EEPROM_MAG_MIN_Z_ADDR, robot.magMin.z);
    EEPROM.put(EEPROM_MAG_MAX_X_ADDR, robot.magMax.x);
    EEPROM.put(EEPROM_MAG_MAX_Y_ADDR, robot.magMax.y);
    EEPROM.put(EEPROM_MAG_MAX_Z_ADDR, robot.magMax.z);
}

inline void loadCompassCalibration(Robot& robot) {
    if (isEEPROMDataValid()) {
        EEPROM.get(EEPROM_COMPASS_OFFSET_ADDR, robot.compassOffset);
        EEPROM.get(EEPROM_COMPASS_CALIBRATED_ADDR, robot.compassCalibrated);
        EEPROM.get(EEPROM_MAG_MIN_X_ADDR, robot.magMin.x);
        EEPROM.get(EEPROM_MAG_MIN_Y_ADDR, robot.magMin.y);
        EEPROM.get(EEPROM_MAG_MIN_Z_ADDR, robot.magMin.z);
        EEPROM.get(EEPROM_MAG_MAX_X_ADDR, robot.magMax.x);
        EEPROM.get(EEPROM_MAG_MAX_Y_ADDR, robot.magMax.y);
        EEPROM.get(EEPROM_MAG_MAX_Z_ADDR, robot.magMax.z);
        if (DEBUG_MODE) Serial.println("Calibration compas chargee depuis l'EEPROM.");
    } else {
        if (DEBUG_MODE) Serial.println("Aucune calibration valide trouvee en EEPROM.");
        robot.compassCalibrated = false;
    }
}

inline bool isEEPROMDataValid() {
    int magicValue = 0;
    EEPROM.get(EEPROM_MAGIC_NUMBER_ADDR, magicValue);
    return (magicValue == EEPROM_MAGIC_VALUE);
}

inline void displayCompassInfo(Robot& robot) {
    float heading = getCalibratedHeading(robot);
    String direction;

    if (heading >= CARDINAL_NORTH_LOWER || heading < CARDINAL_NORTH_UPPER) {
        direction = "N";
    } else if (heading < CARDINAL_NORTHEAST_UPPER) {
        direction = "NE";
    } else if (heading < CARDINAL_EAST_UPPER) {
        direction = "E";
    } else if (heading < CARDINAL_SOUTHEAST_UPPER) {
        direction = "SE";
    } else if (heading < CARDINAL_SOUTH_UPPER) {
        direction = "S";
    } else if (heading < CARDINAL_SOUTHWEST_UPPER) {
        direction = "SO";
    } else if (heading < CARDINAL_WEST_UPPER) {
        direction = "O";
    } else {
        direction = "NO";
    }

    String text = direction + " " + String((int)heading) + "deg";
    setLcdText(robot, text);
}

#endif // COMPASS_H
#ifndef SENSOR_TASK_H
#define SENSOR_TASK_H

#include <Arduino.h>
#include "config.h"
#include "state.h"

// --- Fonctions Publiques ---

// Initialise les broches du capteur à ultrasons.
inline void sensor_init() {
  pinMode(TRIGGER, OUTPUT);
  pinMode(ECHO, INPUT);
}

// Met à jour la mesure du capteur à ultrasons de manière non-bloquante (sans interruptions).
// Cette fonction doit être appelée régulièrement dans la boucle principale.
inline void sensor_update_task(Robot& robot) {
  static unsigned long last_ping_time = 0;
  unsigned long current_time = millis();

  // Envoyer un ping seulement si l'intervalle minimum est écoulé.
  if (current_time - last_ping_time > ULTRASONIC_PING_INTERVAL_MS) {
    last_ping_time = current_time;

    // Envoyer l'impulsion de déclenchement de 10µs pour démarrer une mesure.
    digitalWrite(TRIGGER, LOW);
    delayMicroseconds(ULTRASONIC_TRIGGER_PULSE_LOW_US);
    digitalWrite(TRIGGER, HIGH);
    delayMicroseconds(ULTRASONIC_TRIGGER_PULSE_HIGH_US);
    digitalWrite(TRIGGER, LOW);

    // Mesurer la durée de l'écho avec pulseIn().
    // pulseIn() est bloquant, mais avec un timeout raisonnable, l'impact est limité.
    // C'est une alternative robuste lorsque les interruptions ne sont pas fiables ou disponibles.
    unsigned long duration = pulseIn(ECHO, HIGH, ULTRASONIC_PULSE_TIMEOUT_US);

    if (duration > 0) {
      // La formule standard pour convertir la durée (en microsecondes) en distance (en cm).
      // duration / 2 / 29.1 (vitesse du son) => duration / 58.2
      robot.dusm = duration / ULTRASONIC_DURATION_TO_CM_DIVISOR;
    } else {
      // Si pulseIn() retourne 0, cela signifie qu'aucun écho n'a été reçu avant le timeout.
      robot.dusm = ULTRASONIC_ERROR_VALUE; // Utiliser une valeur négative pour indiquer une erreur ou hors de portée.
    }
  }
}

#endif // SENSOR_TASK_H
