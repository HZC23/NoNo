// --- INCLUDES ---
#include <Arduino.h>

#include <ArduinoJson.h>
#include <Preferences.h> // For storing the active comm mode

#include "config.h"
#include "state.h"
#include "compass.h"
#include "display.h"
#include "terminal.h"
#include "sensor_task.h"
#include "fonctions_motrices.h"
#include "led_fx.h"
#include "support.h"
#include "telemetry.h"
#include "sd_utils.h"
#include "turret_control.h"

#include <Adafruit_NeoPixel.h>

// Include the Xbox controller handler
#include "xbox_controller_bluepad.h"

Adafruit_NeoPixel _earlyLedStrip = Adafruit_NeoPixel(NEOPIXEL_COUNT, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

#if USB_MSC_ENABLED
#include "USB.h"
#include "USBMassStorage.h"

#if USB_MSC_ENABLED
USBMassStorage USBMSC;
bool usbMscActive = false; // Flag to indicate if USB MSD is active
#endif
bool usbMscActive = false; // Flag to indicate if USB MSD is active
#endif


// --- RTOS ---
SemaphoreHandle_t robotMutex = NULL;

// --- BUMPER INTERRUPT ---
volatile bool bumperPressed = false;

// --- HARDWARE OBJECTS DEFINITION ---
MX1508 motorA(AIN1, AIN2);
MX1508 motorB(BIN1, BIN2);
Servo Servodirection; // Ackermann steering
Tourelle tourelle(PINTOURELLE_H, PINTOURELLE_V);
LSM303 *compass;
DFRobot_RGBLCD1602 *lcd;
VL53L1X *vl53;

// --- GLOBAL STATE & COMM OBJECTS ---
Robot robot;
unsigned long startTime;
Preferences preferences;
XboxControllerBluepad xboxController(robot); // Always declare the object
unsigned long lastCompassReadTime = 0;

// --- INTERRUPT SERVICE ROUTINE ---
void IRAM_ATTR onBumperPress() {
  bumperPressed = true;
}


// --- FORWARD DECLARATIONS ---
void updateBatteryStatus(Robot& robot);

void setup() {
    Serial.begin(SERIAL_BAUD_RATE);
    if (DEBUG_MODE) Serial.println("--- NONO BOOTING ---");
    startTime = millis();

    // Init VBAT pin early for initial battery check
    pinMode(VBAT, INPUT);

    // Init early LED strip for boot feedback
    _earlyLedStrip.begin();
    _earlyLedStrip.show(); // Initialize all pixels to 'off'

    // --- CRITICAL EARLY BATTERY CHECK ---
    // If battery is too low, halt here until it's charged.
    // LCD and other I2C devices are not yet initialized.
    static bool ledState = false; // To toggle LED
    while (readBatteryPercentage() < CRITICAL_BATTERY_LEVEL) {
      Serial.println(F("CRITICAL: Batterie trop faible pour initialiser. Chargez SVP!"));

      ledState = !ledState; // Toggle state
      if (ledState) {
        _earlyLedStrip.setPixelColor(0, _earlyLedStrip.Color(255, 0, 0)); // Red
      } else {
        _earlyLedStrip.setPixelColor(0, _earlyLedStrip.Color(0, 0, 0));   // Off
      }
      _earlyLedStrip.show();

      delay(500); // Wait 5 seconds before re-checking (and for blink)
    }
    // Turn off LED after loop exits
    _earlyLedStrip.setPixelColor(0, _earlyLedStrip.Color(0, 0, 0));
    _earlyLedStrip.show();
    Serial.println(F("INFO: Batterie OK pour l'initialisation."));

    // Determine active communication mode from NVS
    bool nvsOpened = preferences.begin(NVS_NAMESPACE, true); // Try read-only first
    if (!nvsOpened) {
        // If read-only failed, try write mode to create it
        nvsOpened = preferences.begin(NVS_NAMESPACE, false);
        if (nvsOpened) {
            // If successfully opened in write mode, save the default value
            preferences.putInt(NVS_COMM_MODE_KEY, COMM_MODE_SERIAL);
            if (DEBUG_MODE) Serial.println("NVS namespace created and default comm mode set.");
        } else {
            // Critical error: NVS still couldn't open
            if (DEBUG_MODE) Serial.println("CRITICAL ERROR: Failed to open NVS namespace. Proceeding with default.");
            // Handle this critical error: for now, continue with the default value,
            // but preferences operations might fail silently or crash later if not checked.
        }
    }
    
    // Now that we've ensured the namespace exists (or tried our best), read the value.
    // If preferences.begin() failed completely, robot.activeCommMode will use the default.
    robot.activeCommMode = (CommunicationMode)preferences.getInt(NVS_COMM_MODE_KEY, COMM_MODE_SERIAL);
    preferences.end();

    // Create a mutex for thread-safe access to the robot object
    robotMutex = xSemaphoreCreateMutex();
    Wire.begin(SDA_PIN, SCL_PIN);
    if (!Wire.begin(SDA_PIN, SCL_PIN)) {
        if (DEBUG_MODE) Serial.println("Erreur d'initialisation I2C");
    }
    Wire.setClock(400000); // Set I2C clock to 400 kHz for faster communication
    delay(500);

    // Instantiate I2C objects AFTER Wire.begin()
    compass = new LSM303();
    lcd = new DFRobot_RGBLCD1602(LCD_I2C_ADDR, LCD_LINE_LENGTH, LCD_ROWS);
    vl53 = new VL53L1X();

    // Initialize SD card
    if (setupSDCard()) {
      robot.sdCardReady = true;
      loadConfig(robot); // Load configuration after SD card is ready
      robot.initialSpeedAvg = robot.speedAvg; // Store initial configured average speed
      robot.initialSpeedSlow = robot.speedSlow; // Store initial configured slow speed
    } else {
      setLcdText(robot, "SD Card Error!");
      while (true);
    }
    delay(500);
    lcd->init();
    
    // --- Initialize Communication Mode based on runtime value ---
    if (robot.activeCommMode == COMM_MODE_XBOX) {
        if (DEBUG_MODE) Serial.println("COMM MODE: Xbox Controller");
        xboxController.begin();
    } else { // Fallback to Serial Only if not Xbox
        if (DEBUG_MODE) Serial.println("COMM MODE: Serial Only");
    }

    setLcdText(robot, LCD_STARTUP_MESSAGE_1);

    // Init hardware
    pinMode(PIR, INPUT);
    pinMode(VBAT, INPUT);
    pinMode(INTERUPTPIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(INTERUPTPIN), onBumperPress, FALLING);
    pinMode(PIN_PHARE, OUTPUT);
    digitalWrite(PIN_PHARE, HIGH);
    
    led_fx_init();

    // Init Sensors & Servos
    sensor_init();
    compass_init(robot);

    if (!vl53->init()) {
      Serial.println(F("CRITICAL: Failed to detect and initialize VL53L1X sensor!"));
      setLcdText(robot, "Laser Error!");
      while (1);
    } else {
      if(DEBUG_MODE) Serial.println("VL53L1X sensor initialized.");
      robot.laserInitialized = true;
      vl53->setDistanceMode(VL53L1X::Long);
      vl53->setMeasurementTimingBudget(robot.laserTimingBudget);
      vl53->startContinuous(robot.laserInterMeasurementPeriod);
    }

    Servodirection.attach(PINDIRECTION);
    Servodirection.write(robot.servoNeutralDir);
    tourelle.attach();
    tourelle.write(robot.servoNeutralTurret, robot.servoNeutralTurret);

    if (robot.activeCommMode == COMM_MODE_XBOX) {
      xboxController.setHardware(motorA, motorB, tourelle);
    }

    if (DEBUG_MODE) Serial.println("--- SETUP COMPLETE ---");
    setLcdText(robot, LCD_STARTUP_MESSAGE_2);
    setLcdText(robot, LCD_STARTUP_MESSAGE_3, true);
    delay(1000);
    digitalWrite(PIN_PHARE, LOW);
}


// --- MAIN LOOP (CORE 1) ---

void loop() {

  robot.loopStartTime = millis();

#if USB_MSC_ENABLED
  if (usbMscActive) {
    // When USB MSD is active, robot logic is paused.
    // The USB stack handles everything.
    // We can add a delay here to reduce CPU usage if needed.
    delay(10); 
    return; 
  }
#endif

  if(xSemaphoreTake(robotMutex, (TickType_t) MUTEX_WAIT_TICKS) == pdTRUE) {

    // --- High-Priority Interrupt Checks ---
    if (bumperPressed) {
      bumperPressed = false; // Reset flag immediately
      if (robot.currentState != EMERGENCY_EVASION) {
        changeState(robot, EMERGENCY_EVASION);
      }
    }

    // --- Turret Control ---
    bool isMoving = (robot.currentState == MOVING_FORWARD || robot.currentState == FOLLOW_HEADING || robot.currentState == MAINTAIN_HEADING);
    updateTurret(robot, isMoving);

    // --- Process input based on active communication mode ---
    if (robot.activeCommMode == COMM_MODE_XBOX) {
      xboxController.processControllers();
    }
    
    // Always check for serial commands for debugging and mode switching
    checkSerial();

    updateBatteryStatus(robot);

    // --- CRITICAL BATTERY HANDLING ---
    if (robot.batteryIsCritical) {
      // This is a blocking loop that holds the robot until it's recharged.
      Arret(); // Stop motors immediately.
      Serial.println("CRITICAL: Batterie Vide. Robot en pause jusqu'a recharge.");
      bool criticalMessageDisplayed = false; // Flag to control LCD message

      while (robot.batteryIsCritical) {
        // 1. Display message on LCD (only once)
        if (!criticalMessageDisplayed) {
          setLcdText(robot, "Batterie Vide!");
          criticalMessageDisplayed = true;
        }
        handleLcdAnimations(robot); // ALWAYS process the animation engine for the text

        // 2. Update LED effects (will show critical battery)
        led_fx_update(robot);

        // 3. Continuously re-check battery status
        updateBatteryStatus(robot);

        // 4. Small delay to prevent this loop from running too fast
        delay(250); 
      }
      
      // Once the loop is broken, it means the battery is OK.
      Serial.println("INFO: Batterie OK. Reprise des operations.");
      setLcdText(robot, "Batterie OK !");
      handleLcdAnimations(robot);
      delay(1000); // Let the "OK" message be seen
    }

    // Update sensors
    sensor_update_task(robot);
    if (robot.currentState != IDLE || (millis() - lastCompassReadTime > COMPASS_READ_INTERVAL_MS)) {
      robot.cap = getCalibratedHeading(robot);
      lastCompassReadTime = millis();
    }
    robot.currentPitch = getPitch(robot);
    if (robot.laserInitialized && vl53->dataReady()) {
      robot.distanceLaser = vl53->readRangeContinuousMillimeters() / MM_TO_CM_DIVISOR;
    }
    
    // Update LCD display
    displayJokesIfIdle(robot);
    updateLcdDisplay(robot);
    handleLcdAnimations(robot);

    // Initial autonomous action
    if (!robot.initialActionTaken && robot.currentState == IDLE && millis() - startTime >= robot.initialAutonomousDelay) {
        changeState(robot, OBSTACLE_AVOIDANCE);
        robot.initialActionTaken = true;
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

    // Send telemetry periodically
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

  // Handle critical level first - this is an immediate stop condition
  if (batteryLevel < CRITICAL_BATTERY_LEVEL) {
    if (!robot.batteryIsCritical) {
      robot.batteryIsCritical = true;
      robot.batteryIsLow = true; // A critical battery is also a low battery
    }
    return; // Exit immediately, no further checks
  }

  // --- Hysteresis Logic ---
  // If we are currently in a low or critical state...
  if (robot.batteryIsLow || robot.batteryIsCritical) {
    // ...we need to reach the higher RECHARGED_THRESHOLD to recover.
    if (batteryLevel >= RECHARGED_THRESHOLD) {
      robot.speedAvg = robot.initialSpeedAvg; // Restore initial average speed
      robot.speedSlow = robot.initialSpeedSlow; // Restore initial slow speed
      robot.batteryIsLow = false;
      robot.batteryIsCritical = false; // Should be false already but set for safety
    }
  } 
  // If we are not currently in a low battery state...
  else {
    // ...we enter the low battery state only if we drop below the lower threshold.
    if (batteryLevel < LOW_BATTERY_THRESHOLD) {
      if (!robot.batteryIsLow) {
        robot.speedAvg = robot.speedSlow; 
        robot.speedSlow = robot.speedSlow / 2; 
        robot.batteryIsLow = true;
      }
    }
  }
}
