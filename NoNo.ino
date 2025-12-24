// --- INCLUDES ---
#include <Arduino.h>
#include "robot.h"
#include "hardware.h"
#include "comms.h"
#include "fonctions_motrices.h"

#if USB_MSC_ENABLED
#include "USB.h"
#include "USBMassStorage.h"
USBMassStorage USBMSC;
bool usbMscActive = false;
#endif

// --- SETUP ---
void setup() {
    Serial.begin(SERIAL_BAUD_RATE);
    if (DEBUG_MODE) Serial.println("--- NONO BOOTING ---");
    robot.startTime = millis();

    pinMode(VBAT, INPUT);
    pixels.begin();
    pixels.show();

    while (readBatteryPercentage() < CRITICAL_BATTERY_LEVEL) {
      Serial.println(F("CRITICAL: Batterie trop faible pour initialiser. Chargez SVP!"));
      pixels.setPixelColor(0, (millis() % 1000 < 500) ? pixels.Color(255, 0, 0) : 0);
      pixels.show();
      delay(500);
    }
    pixels.setPixelColor(0, 0);
    pixels.show();
    Serial.println(F("INFO: Batterie OK pour l'initialisation."));

    preferences.begin(NVS_NAMESPACE, false);
    robot.activeCommMode = (CommunicationMode)preferences.getInt(NVS_COMM_MODE_KEY, COMM_MODE_SERIAL);
    preferences.end();

    robotMutex = xSemaphoreCreateMutex();
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(400000);
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
        if (DEBUG_MODE) Serial.println("COMM MODE: Xbox Controller");
        xboxController.begin();
    } else {
        if (DEBUG_MODE) Serial.println("COMM MODE: Serial Only");
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
    delay(1000);
    setLcdText(robot, LCD_STARTUP_MESSAGE_3);
    delay(1000);
    digitalWrite(PIN_PHARE, LOW);
}


// --- MAIN LOOP ---
void loop() {
  robot.loopStartTime = millis();

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
      Serial.println("CRITICAL: Batterie Vide. Robot en pause jusqu'a recharge.");
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
      Serial.println("INFO: Batterie OK. Reprise des operations.");
      setLcdText(robot, "Batterie OK !");
      delay(1000);
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

    if (!robot.initialActionTaken && robot.currentState == IDLE && millis() - robot.startTime >= robot.initialAutonomousDelay) {
        changeState(robot, OBSTACLE_AVOIDANCE);
        robot.initialActionTaken = true;
    }

    if (millis() - robot.lastAppCommandTime < 100) {
        // Manual command override
    } else {
        updateMotorControl(robot);
    }

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
