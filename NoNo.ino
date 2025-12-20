// --- INCLUDES ---
#include <Arduino.h>
#include <ArduinoJson.h> // Required for telemetry

#include "config.h"      // Central configuration for pins and constants
#include "state.h"       // Global state and hardware objects
#include "compass.h"     // Compass logic
#include "display.h"     // LCD display functions
#include "terminal.h"    // Serial command processing
#include "sensor_task.h" // Ultrasonic sensor task
#include "fonctions_motrices.h" // Motor control state machine
#include "led_fx.h"      // NeoPixel LED effects
#include "support.h"     // Misc support functions
#include "telemetry.h"   // For sending JSON data
#include "sd_utils.h"    // For SD card utilities
#include "xbox_controller_bluepad.h" // Include the new Xbox controller library

// --- RTOS ---
SemaphoreHandle_t robotMutex = NULL;

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

// --- XBOX CONTROLLER OBJECT ---
XboxControllerBluepad xboxController(robot);

// --- FORWARD DECLARATIONS ---
void updateBatteryStatus(Robot& robot);

void setup() {
    Serial.begin(SERIAL_BAUD_RATE);
    if (DEBUG_MODE) Serial.println("--- NONO BOOTING ---");
    startTime = millis();

    // Create a mutex for thread-safe access to the robot object
    robotMutex = xSemaphoreCreateMutex();
    Wire.begin(SDA_PIN, SCL_PIN); // Initialize I2C
    if (!Wire.begin(SDA_PIN, SCL_PIN)) {
        if (DEBUG_MODE) Serial.println("Erreur d'initialisation I2C");
    }
    Wire.setTimeOut(50); // Prevent I2C freezes
    delay(500);
    
    // Initialize SD card and update robot state
    robot.sdCardReady = setupSDCard();
    if (!robot.sdCardReady) {
      // Handle SD card error without blocking the entire robot
      setLcdText(robot, "SD Card Error!");
      delay(2000); // Display the error for 2 seconds
    }
    delay(500);
    // Initialize Xbox controller (Bluepad32)
    xboxController.begin();
    
    setLcdText(robot, LCD_STARTUP_MESSAGE_1); // Keep the first startup message for a moment

    // Init hardware
    pinMode(PIR, INPUT);
    pinMode(VBAT, INPUT);
    pinMode(INTERUPTPIN, INPUT_PULLUP);
    pinMode(PIN_PHARE, OUTPUT); // Initialize headlight pin
    
    led_fx_init();

    // Init Sensors & Servos
    sensor_init();
    compass_init(robot);
    robot.compassInverted = COMPASS_IS_INVERTED; // Apply inversion setting from config

    // Initialize configurable parameters
    robot.speedAvg = VITESSE_MOYENNE;
    robot.speedSlow = VITESSE_LENTE;

    // Servodirection.attach(PINDIRECTION, 70, 105); // Ackermann steering
    // Servodirection.write(NEUTRE_DIRECTION);      // Ackermann steering
      tourelle.attach();
      tourelle.write(SCAN_CENTER_ANGLE, NEUTRE_TOURELLE);

    // Pass global hardware objects to the XboxControllerBluepad instance
    xboxController.setHardware(motorA, motorB, tourelle);

    if (DEBUG_MODE) Serial.println("--- SETUP COMPLETE ---");
    setLcdText(robot, LCD_STARTUP_MESSAGE_2);
    delay(SCROLL_DELAY_MS); // Show second startup message for 2 seconds
    digitalWrite(PIN_PHARE, LOW); // Turn off headlight at the end of setup
}


// --- MAIN LOOP (CORE 1) ---

void loop() {

  robot.loopStartTime = millis();

  if(xSemaphoreTake(robotMutex, (TickType_t) MUTEX_WAIT_TICKS) == pdTRUE) {

    // Process controller input
    xboxController.processControllers();

    updateBatteryStatus(robot);
    led_fx_update(robot); // Update NeoPixel LEDs

    // Update sensors
    sensor_update_task(robot);
    robot.cap = getCalibratedHeading(robot);
    robot.currentPitch = getPitchAngle(robot); // Update current pitch for horizon stabilization
    if (robot.laserInitialized && vl53.dataReady()) {
      robot.distanceLaser = vl53.readRangeContinuousMillimeters() / MM_TO_CM_DIVISOR;
    }
    
    // Update LCD display with current status or jokes if idle
    displayJokesIfIdle(robot);
    updateLcdDisplay(robot);

    // Transition to OBSTACLE_AVOIDANCE only once at startup
    if (!robot.initialActionTaken && robot.currentState == IDLE && millis() - startTime >= INITIAL_AUTONOMOUS_DELAY_MS) {
        changeState(robot, OBSTACLE_AVOIDANCE);
        robot.currentNavMode = AUTONOMOUS_CONTROL; // Set navigation mode to autonomous
        robot.initialActionTaken = true;
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
        // Do nothing here, music is played in a separate task
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
    
    xSemaphoreGive(robotMutex);
  }

  // Regulate loop frequency
  robot.loopEndTime = millis();
  unsigned long loopDuration = robot.loopEndTime - robot.loopStartTime;
  if (loopDuration < LOOP_TARGET_PERIOD_MS) {
    delay(LOOP_TARGET_PERIOD_MS - loopDuration);
  }
}


void updateBatteryStatus(Robot& robot) {
  int batteryLevel = readBatteryPercentage();

  // Handle critical battery level (e.g., completely empty)
  if (batteryLevel < CRITICAL_BATTERY_LEVEL) { // Using < 1 instead of == 0 for robustness
    if (!robot.batteryIsCritical) {
      robot.batteryIsCritical = true;
      robot.batteryIsLow = true; // Critical implies low
    }
    return; // Stop further evaluation
  }

  // Handle low battery threshold
  if (batteryLevel < LOW_BATTERY_THRESHOLD) {
    if (!robot.batteryIsLow) {
      // First time hitting low battery
      robot.speedAvg = VITESSE_LENTE; // Slow down
      robot.speedSlow = VITESSE_LENTE / 2;
      robot.batteryIsLow = true;
    }
    // Note: We don't set batteryIsCritical to false here. 
    // It should only be cleared when battery is well above critical.
  } 
  // Handle battery recovery
  else {
    // Check if either flag was active
    if (robot.batteryIsLow || robot.batteryIsCritical) {
      // Restore default speeds
      robot.speedAvg = VITESSE_MOYENNE;
      robot.speedSlow = VITESSE_LENTE;
      robot.batteryIsLow = false;
      robot.batteryIsCritical = false; // Reset both flags
    }
  }
}