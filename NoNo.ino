// --- INCLUDES ---
#include <Arduino.h>
#include <cmath>
#include "robot.h"
#include "logger.h"
#include "config.h"
#include "hardware.h"
#include "battery_utils.h"
#include "comms.h"
#include "fonctions_motrices.h"

// --- Global Robot Instance Definition ---
Robot robot;

// --- Function to initialize robot default values ---
void initializeRobotDefaults(Robot& robot) {
    robot.speedAvg = VITESSE_MOYENNE;
    robot.targetSpeed = robot.speedAvg; // Set master speed from config
    robot.speedSlow = VITESSE_LENTE;
    robot.speedRotation = VITESSE_ROTATION;
    robot.speedRotationMax = VITESSE_ROTATION_MAX;
    robot.turnTolerance = TOLERANCE_VIRAGE;
    robot.KpHeading = Kp_HEADING;
    robot.motorBCalibration = CALIBRATION_MOTEUR_B;
    robot.minSpeedToMove = MIN_SPEED_TO_MOVE;
    robot.accelRate = ACCEL_RATE;
    robot.diffStrength = DIFF_STRENGTH;
    robot.fwdDiffCoeff = FWD_DIFF_COEFF;
    robot.servoNeutralDir = NEUTRE_DIRECTION;
    robot.servoNeutralTurret = NEUTRE_TOURELLE;
    robot.servoDirMin = SERVO_DIR_MIN;
    robot.servoDirMax = SERVO_DIR_MAX;
    robot.servoAngleHeadDown = ANGLE_TETE_BASSE;
    robot.servoAngleGround = ANGLE_SOL;
    robot.avoidBackupDuration = AVOID_BACKUP_DURATION_MS;
    robot.minDistForValidPath = MIN_DIST_FOR_VALID_PATH;
    robot.turretMoveTime = TURRET_MOVE_TIME_MS;
    robot.turretScanDelay = SCAN_DELAY_MS;
    robot.anglePenaltyFactor = ANGLE_PENALTY_FACTOR;
    robot.seuilVide = SEUIL_VIDE;
    robot.laserTimingBudget = VL53L1X_TIMING_BUDGET_US;
    robot.laserInterMeasurementPeriod = VL53L1X_INTER_MEASUREMENT_PERIOD_MS;
    robot.initialAutonomousDelay = INITIAL_AUTONOMOUS_DELAY_MS;
    robot.pivotAngleThreshold = SEUIL_BASCULE_DIRECTION;
    robot.compassInverted = COMPASS_IS_INVERTED;
    robot.currentState = IDLE;
}

// --- SETUP ---
void setup() {
    Serial.begin(SERIAL_BAUD_RATE);
    logger_init(DEBUG_MODE ? LOG_LEVEL_DEBUG : LOG_LEVEL_INFO); // Initialize logger based on DEBUG_MODE
    LOG_INFO("--- NONO BOOTING ---");
    
    initializeRobotDefaults(robot); // Initialize robot state with default values
    robot.startTime = millis();

    pinMode(VBAT, INPUT);
    analogSetPinAttenuation(VBAT, ADC_11db); // Set attenuation for 0-3.3V range
    
    led_fx_init(); // Initialize LEDs early

    // Try to recover the I2C bus before initializing devices
    clearI2CBus();

    // Initialize I2C and LCD early for debugging the battery check.
    Wire.begin(SDA_PIN, SCL_PIN);
    scanI2CBus(); // Call I2C scanner
    lcd = new DFRobot_RGBLCD1602(LCD_I2C_ADDR, LCD_LINE_LENGTH, LCD_ROWS);
    lcd->init();
    lcd->setBacklight(true);

    while (readBatteryPercentage() < CRITICAL_BATTERY_LEVEL) {
      LOG_ERROR("Batterie trop faible pour initialiser. Chargez SVP!");
      
      // Display current voltage on the LCD for debugging
      float voltage = readBatteryVoltage();
      char buffer[16];
      sprintf(buffer, "Batt: %.2fV", voltage);
      lcd->clear();
      lcd->print(buffer);

      // Flash red on all LEDs using the new system logic if possible, 
      // or just manual for this very early boot stage if robot isn't fully ready.
      led_fx_set_all( (millis() % 1000 < 500) ? 255 : 0, 0, 0);
      delay(1000);
    }
    led_fx_off();
    LOG_INFO("Batterie OK pour l'initialisation.");

    preferences.begin(NVS_NAMESPACE, false);
    robot.activeCommMode = (CommunicationMode)preferences.getInt(NVS_COMM_MODE_KEY, COMM_MODE_SERIAL);
    preferences.end();

    robotMutex = xSemaphoreCreateMutex();
    // Wire.begin(SDA_PIN, SCL_PIN); // Already called
    delay(500);

    compass = new LSM303();
    // lcd = new DFRobot_RGBLCD1602(LCD_I2C_ADDR, LCD_LINE_LENGTH, LCD_ROWS); // Already created
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
    // lcd->init(); // Already called
    
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

    // The turret servo (tourelle) is attached/detached automatically by the changeState function
    // to manage power. The direction servo (Servodirection) is attached manually here
    // and remains attached for immediate control.
    Servodirection.attach(PINDIRECTION);
    Servodirection.write(robot.servoNeutralDir);

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

  if(xSemaphoreTake(robotMutex, (TickType_t) MUTEX_WAIT_TICKS) == pdTRUE) {
    if (bumperPressed) {
      bumperPressed = false;
      // Do not trigger emergency evasion if in manual mode
      if (robot.currentState != EMERGENCY_EVASION && robot.currentState != MANUAL_COMMAND_MODE) {
        trigger_rumble(100, 0, 255); // Strong rumble for bumper press
        robot.stateBeforeEvasion = robot.currentState;
        changeState(robot, EMERGENCY_EVASION, AVOID_IDLE);
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
      float heading = getCalibratedHeading(robot);
      // Only update heading if compass is initialized (value is not NaN)
      if (!std::isnan(heading)) {
        robot.cap = heading;
      }
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

// #if USB_MSC_ENABLED
// void activate_msc_mode() {
//     if (usbMscActive) return;

//     LOG_INFO("Activating USB Mass Storage mode.");
//     setLcdText(robot, "USB Mode Active");
//     Arret(); // Stop motors

//     SdFat* sd_card = get_sd_card();
//     if (!sd_card) {
//         LOG_ERROR("Could not get SD card filesystem to start MSC.");
//         setLcdText(robot, "SD Card Error!");
//         return;
//     }

//     USBMSC.setID("Nono", "SD Card", "1.0");
    
//     // Set callbacks
//     USBMSC.setReadWriteCallback(msc_read_cb, msc_write_cb, msc_flush_cb);
//     USBMSC.setReadyCallback(msc_ready_cb);

//     // Set disk size. sd_card->card() returns a pointer to the SdCard object.
//     // sectorCount() returns the number of sectors on the card.
//     // The block size is almost always 512 bytes for SD cards.
//     uint32_t block_count = sd_card->card()->sectorCount();
//     USBMSC.setCapacity(block_count, 512);
//     USBMSC.setUnitReady(true);

//     if (USBMSC.begin()) {
//         USBDevice.begin();
//         usbMscActive = true;
//         LOG_INFO("USB MSC Started. Robot will be unresponsive until reset.");
//     } else {
//         LOG_ERROR("Failed to begin USB MSC.");
//         setLcdText(robot, "USB MSC Error!");
//     }
// }

// // MSC callback implementations
// int32_t msc_read_cb(uint32_t lba, void *buffer, uint32_t bufsize) {
//     SdFat* sd_card = get_sd_card();
//     if (!sd_card) return -1;

//     // sd_card->card() returns a pointer to the SdCard object, which has low-level read/write functions.
//     // readBlocks takes LBA, buffer, and number of blocks. bufsize is in bytes.
//     return sd_card->card()->readBlocks(lba, (uint8_t *)buffer, bufsize / 512) ? (int32_t)bufsize : -1;
// }

// int32_t msc_write_cb(uint32_t lba, uint8_t *buffer, uint32_t bufsize) {
//     SdFat* sd_card = get_sd_card();
//     if (!sd_card) return -1;
    
//     // writeBlocks takes LBA, buffer, and number of blocks.
//     return sd_card->card()->writeBlocks(lba, buffer, bufsize / 512) ? (int32_t)bufsize : -1;
// }

// void msc_flush_cb(void) {
//     SdFat* sd_card = get_sd_card();
//     if (sd_card) {
//         // syncBlocks() ensures all cached data is written to the card.
//         sd_card->card()->syncBlocks();
//     }
// }

// bool msc_ready_cb(void) {
//     // The device is ready if we have a valid pointer to the SD card object.
//     return (get_sd_card() != nullptr);
// }
// #endif

