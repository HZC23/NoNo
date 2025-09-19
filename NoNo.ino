// --- INCLUDES ---
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_VL53L1X.h>
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
Servo Servodirection;
Tourelle tourelle(PINTOURELLE_H, PINTOURELLE_V);
LSM303 compass;
DFRobot_RGBLCD1602 lcd(0x60, 16, 2);
Adafruit_VL53L1X vl53 = Adafruit_VL53L1X();

// --- GLOBAL STATE OBJECT ---
Robot robot;

// --- SETUP ---
void setup() {
    Serial.begin(115200);
    Wire.begin();
    lcd.init();
    if (DEBUG_MODE) Serial.println("--- NONO STARTUP ---");

    if (!vl53.begin(0x29, &Wire)) {
        Serial.println(F("Failed to boot VL53L1X"));
        while(1);
    }
    vl53.startContinuous(50);

    setLcdText(robot, "Je suis NONO");

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
    Servodirection.attach(PINDIRECTION, 70, 105);
    Servodirection.write(NEUTRE_DIRECTION);
    #if ENABLE_TOWER
      tourelle.attach();
      tourelle.write(SCAN_CENTER_ANGLE, 90);
    #endif

    PhareAllume(); // Turn on headlights at startup

    if (DEBUG_MODE) Serial.println("--- SETUP COMPLETE ---");
    setLcdText(robot, "Pret.");
}

// --- MAIN LOOP ---
void loop() {
  // Process incoming commands
  Terminal(robot); 

  // Update sensors
  sensor_update_task(robot);
  robot.cap = getCalibratedHeading(robot);
  if (vl53.dataReady()) {
    robot.distanceLaser = vl53.distance();
    vl53.clearInterrupt();
  }

  // Run state machines
  switch (robot.currentState) {
    case CALIBRATING_COMPASS:
      calibrateCompass(robot);
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