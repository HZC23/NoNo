// --- INCLUDES ---
#include <Arduino.h>
#include "robot.h"
#include "hardware.h"
#include "comms.h"
#include "fonctions_motrices.h"
#include "logger.h" // Include the new logger

#if USB_MSC_ENABLED
#include "USB.h"
#include "USBMassStorage.h"
USBMassStorage USBMSC;
bool usbMscActive = false;
#endif

// --- SETUP ---
void setup() {
    Serial.begin(SERIAL_BAUD_RATE);
    logger_init(DEBUG_MODE ? LOG_LEVEL_DEBUG : LOG_LEVEL_INFO); // Initialize logger based on DEBUG_MODE
    LOG_INFO("--- NONO BOOTING ---");
    robot.startTime = millis();

    pinMode(VBAT, INPUT);
    pixels.begin();
    pixels.show();

    while (readBatteryPercentage() < CRITICAL_BATTERY_LEVEL) {
      LOG_ERROR("Batterie trop faible pour initialiser. Chargez SVP!");
      pixels.setPixelColor(0, (millis() % 1000 < 500) ? pixels.Color(255, 0, 0) : 0);
      pixels.show();
      delay(500);
    }
    pixels.setPixelColor(0, 0);
    pixels.show();
    LOG_INFO("Batterie OK pour l'initialisation.");

    preferences.begin(NVS_NAMESPACE, false);
    robot.activeCommMode = (CommunicationMode)preferences.getInt(NVS_COMM_MODE_KEY, COMM_MODE_SERIAL);
    preferences.end();

    robotMutex = xSemaphoreCreateMutex();
    Wire.begin(SDA_PIN, SCL_PIN);
    delay(500);

    compass = new LSM303();
    lcd = new DFRobot_RGBLCD1602(LCD_I2C_ADDR, LCD_LINE_LENGTH, LCD_ROWS);
    vl53 = new VL53L1X();

    if (setupSDCard()) {
      robot.sdCardReady = true;
      loadConfig(robot); 
      robot.initialSpeedAvg = robot.speedAvg;
      robot.initialSpeedSlow = robot.speedSlow;
    } else {
      // This part requires the LCD to be initialized to show the message.
      // lcd->init();
      // setLcdText(robot, "SD Card Error!");
      // while (true);
    }
    lcd->init();
    
    if (robot.activeCommMode == COMM_MODE_XBOX) {
        LOG_INFO("COMM MODE: Xbox Controller");
        xboxController.begin();
    } else {
        LOG_INFO("COMM MODE: Serial Only");
    }

    setLcdText(robot, LCD_STARTUP_MESSAGE_1);

    pinMode(PIR, INPUT);
    pinMode(INTERUPTPIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(INTERUPTPIN), onBumperPress, FALLING);
    pinMode(PIN_PHARE, OUTPUT);
    digitalWrite(PIN_PHARE, HIGH);
    
    led_fx_init();
    sensor_init();
    compass_init(robot);

    if (!vl53->init()) {
      LOG_ERROR("Failed to detect and initialize VL53L1X sensor!");
      setLcdText(robot, "Laser Error!");
      while (1);
    } else {
      LOG_INFO("VL53L1X sensor initialized.");
      robot.laserInitialized = true;
      vl53->setDistanceMode(VL53L1X::Long);
      vl53->setMeasurementTimingBudget(robot.laserTimingBudget);
      vl53->startContinuous(robot.laserInterMeasurementPeriod);
    }
    Wire.setClock(400000);

    Servodirection.attach(PINDIRECTION);
    Servodirection.write(robot.servoNeutralDir);
    tourelle.attach();
    tourelle.write(robot.servoNeutralTurret, robot.servoNeutralTurret);

    LOG_INFO("--- SETUP COMPLETE ---");
    setLcdText(robot, LCD_STARTUP_MESSAGE_2);
    delay(1000);
    setLcdText(robot, LCD_STARTUP_MESSAGE_3);
    delay(1000);
    digitalWrite(PIN_PHARE, LOW);
}


// --- MAIN LOOP ---
void loop() {
  robot.loopStartTime = millis();

  // Debug prints for diagnostics
  static unsigned long lastDebugPrintTime = 0;
  if (millis() - lastDebugPrintTime > 1000) { // Print every second
    LOG_DEBUG("bumperPressed=%d, activeCommMode=%d, currentState=%d", bumperPressed, robot.activeCommMode, robot.currentState);
    lastDebugPrintTime = millis();
  }

#if USB_MSC_ENABLED
  if (usbMscActive) {
    delay(10); 
    return; 
  }
#endif

  if(xSemaphoreTake(robotMutex, (TickType_t) MUTEX_WAIT_TICKS) == pdTRUE) {

    if (bumperPressed) {
      bumperPressed = false;
      if (robot.currentState != EMERGENCY_EVASION) {
        trigger_rumble(100, 0, 255); // Strong rumble for bumper press
        robot.stateBeforeEvasion = robot.currentState;
        changeState(robot, EMERGENCY_EVASION);
      }
    }

    bool isMoving = (robot.currentState == MOVING_FORWARD || robot.currentState == FOLLOW_HEADING || robot.currentState == MAINTAIN_HEADING);
    updateTurret(robot, isMoving);

    if (robot.activeCommMode == COMM_MODE_XBOX) {
      xboxController.processControllers();
    }
    
    checkSerial();
    updateBatteryStatus(robot);

    if (robot.batteryIsCritical) {
      Arret();
      LOG_ERROR("Batterie Vide. Robot en pause jusqu'a recharge.");
      bool criticalMessageDisplayed = false;
      while (robot.batteryIsCritical) {
        if (!criticalMessageDisplayed) {
          setLcdText(robot, "Batterie Vide!");
          criticalMessageDisplayed = true;
        }
        handleLcdAnimations(robot);
        led_fx_update(robot);
        updateBatteryStatus(robot);
        delay(250); 
      }
    }

    sensor_update_task(robot);
    if (robot.currentState != IDLE || (millis() - robot.lastCompassReadTime > COMPASS_READ_INTERVAL_MS)) {
      robot.cap = getCalibratedHeading(robot);
      robot.lastCompassReadTime = millis();
    }
    robot.currentPitch = getPitch(robot);
    if (robot.laserInitialized && vl53->dataReady()) {
      robot.distanceLaser = vl53->readRangeContinuousMillimeters() / MM_TO_CM_DIVISOR;
    }
    
    displayJokesIfIdle(robot);
    updateLcdDisplay(robot);
    handleLcdAnimations(robot);
    led_fx_update(robot); // Update LED effects

    if (!robot.initialActionTaken && robot.currentState == IDLE && millis() - robot.startTime >= robot.initialAutonomousDelay) {
        changeState(robot, OBSTACLE_AVOIDANCE);
        robot.initialActionTaken = true;
    }

    updateMotorControl(robot);

    if (millis() - robot.lastReportTime > robot.reportInterval) {
      robot.lastReportTime = millis();
      sendTelemetry(robot);
    }
    
    xSemaphoreGive(robotMutex);
  }

  robot.loopEndTime = millis();
  unsigned long loopDuration = robot.loopEndTime - robot.loopStartTime;
  if (loopDuration < LOOP_TARGET_PERIOD_MS) {
    delay(LOOP_TARGET_PERIOD_MS - loopDuration);
  }
}
