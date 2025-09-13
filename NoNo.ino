
// --- INCLUDES ---
#include <Arduino.h>
#include <Wire.h>
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

// --- HARDWARE OBJECTS DEFINITION ---
// These are defined here and declared as 'extern' in state.h
MX1508 motorA(AIN1, AIN2);
MX1508 motorB(BIN1, BIN2);
Servo Servodirection;
Tourelle tourelle(PINTOURELLE_H, PINTOURELLE_V);
LSM303 compass;
DFRobot_RGBLCD1602 lcd(0x60, 16, 2);

// --- GLOBAL STATE OBJECT ---
// All state variables are now encapsulated in this single object
Robot robot;

// --- SETUP ---
void setup() {
    Serial.begin(115200);
    Wire.begin();
    lcd.init();
    if (DEBUG_MODE) Serial.println("--- NONO STARTUP ---");

    setLcdText(robot, "Je suis NONO");

    // Init hardware
    pinMode(PIR, INPUT);
    pinMode(VBAT, INPUT);
    pinMode(INTERUPTPIN, INPUT_PULLUP);
    
    // Init LEDs
    #if ENABLE_LEDS
      pinMode(LED_ROUGE, OUTPUT);
      pinMode(LED_JAUNE, OUTPUT);
      digitalWrite(LED_ROUGE, LOW);
      digitalWrite(LED_JAUNE, LOW);
    #endif

    // Init Sensors
    sensor_init(); // For ultrasonic
    compass_init(robot); // For compass

    // Init Servos
    Servodirection.attach(PINDIRECTION, 70, 105);
    Servodirection.write(NEUTRE_DIRECTION);
    #if ENABLE_TOWER
      tourelle.attach();
      tourelle.write(SCAN_CENTER_ANGLE, 90);
    #endif

    PhareAllume(); // from support.h
    Batterie();    // from support.h

    if (DEBUG_MODE) Serial.println("--- SETUP COMPLETE ---");
    setLcdText(robot, "Pret.");
}

// --- MAIN LOOP ---
void loop() {
  // Always listen for commands
  Terminal(robot); 

  // Non-blocking sensor update
  sensor_update_task(robot);

  // Main state machine
  switch (robot.currentState) {
    case CALIBRATING_COMPASS:
      calibrateCompass(robot);
      break;

    default:
      updateMotorControl(robot);
      // Mcap(10); // This function needs to be refactored
      // sendPeriodicData(); // This function needs to be refactored
      break;
  }
}
