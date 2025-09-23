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

// --- HARDWARE OBJECTS DEFINITION ---
MX1508 motorA(AIN1, AIN2);
MX1508 motorB(BIN1, BIN2);
// Servo Servodirection; // Ackermann steering
Tourelle tourelle(PINTOURELLE_H, PINTOURELLE_V);
LSM303 compass;
DFRobot_RGBLCD1602 lcd(0x60, 16, 2);
VL53L1X vl53;

// --- GLOBAL STATE OBJECT ---
Robot robot;
unsigned long startTime;

// --- FORWARD DECLARATIONS ---
void handleScanning(Robot& robot);

// --- SETUP ---
void setup() {
    Serial.begin(115200);
    Wire.begin();
    lcd.init();
    if (DEBUG_MODE) Serial.println("--- NONO STARTUP ---");
    startTime = millis(); // Initialize startupTime

    if (DEBUG_MODE) Serial.println("Initializing VL53L1X...");
    vl53.setBus(&Wire);
    if (!vl53.init()) {
        Serial.println(F("Failed to boot VL53L1X"));
        while(1);
    }
    if (DEBUG_MODE) Serial.println("VL53L1X Initialized.");
    vl53.setMeasurementTimingBudget(50000);
    vl53.startContinuous(50);

    setLcdText(robot, LCD_STARTUP_MESSAGE_1);

    // Init hardware
    pinMode(PIR, INPUT);
    pinMode(VBAT, INPUT);
    pinMode(INTERUPTPIN, INPUT_PULLUP);
    pinMode(PIN_PHARE, OUTPUT); // Initialize headlight pin
    
    #if ENABLE_LEDS
      pinMode(LED_ROUGE, OUTPUT);
      pinMode(LED_JAUNE, OUTPUT);
    #endif

    // Init Sensors & Servos
    sensor_init();
    compass_init(robot);
    robot.compassInverted = COMPASS_IS_INVERTED; // Apply inversion setting from config
    // Servodirection.attach(PINDIRECTION, 70, 105); // Ackermann steering
    // Servodirection.write(NEUTRE_DIRECTION);      // Ackermann steering
    #if ENABLE_TOWER
      tourelle.attach();
      tourelle.write(SCAN_CENTER_ANGLE, 90);
    #endif

    PhareAllume(); // Turn on headlights at startup

    if (DEBUG_MODE) Serial.println("--- SETUP COMPLETE ---");
    setLcdText(robot, LCD_STARTUP_MESSAGE_2);
}

// --- MAIN LOOP ---
void loop() {
  // Process incoming commands
  Terminal(robot); 

  // Transition to OBSTACLE_AVOIDANCE only once at startup
  if (!robot.initialActionTaken && robot.currentState == IDLE && millis() - startTime >= INITIAL_AUTONOMOUS_DELAY_MS) {
      changeState(robot, OBSTACLE_AVOIDANCE);
      robot.initialActionTaken = true;
  }

  // Update sensors
  sensor_update_task(robot);
  robot.cap = getCalibratedHeading(robot);
  if (vl53.dataReady()) {
    robot.distanceLaser = vl53.readRangeContinuousMillimeters() / 10;
  }

  // Run state machines
  switch (robot.currentState) {
    case CALIBRATING_COMPASS:
      calibrateCompass(robot);
      break;

    case SCANNING: // Unified scanning state
      handleScanning(robot);
      break;

    default:
      updateMotorControl(robot);
      break;
  }

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
    tourelle.write(robot.currentScanAngleH, 90); // Start scan at 90 deg vertical
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
      tourelle.write(SCAN_CENTER_ANGLE, 90); // Center turret when done
      changeState(robot, IDLE); // Scan complete
      if (DEBUG_MODE) Serial.println(F("Scan complete."));
    } else {
      tourelle.write(robot.currentScanAngleH, 90);
    }
  }
}
