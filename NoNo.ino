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

// --- SETUP ---
void setup() {
    Serial.begin(SERIAL_BAUD_RATE);
    logger_init(DEBUG_MODE ? LOG_LEVEL_DEBUG : LOG_LEVEL_INFO); // Initialize logger based on DEBUG_MODE
    LOG_INFO("--- NONO BOOTING ---");
    
    initializeRobot(robot); // Initialize robot state with default values
    robot.startTime = millis();

    pinMode(VBAT, INPUT);
    analogSetPinAttenuation(VBAT, ADC_11db); // Set attenuation for 0-3.3V range
    
    led_fx_init(); // Initialize LEDs early

    // Try to recover the I2C bus before initializing devices
    clearI2CBus();

    // Initialize I2C and LCD early for debugging the battery check.
    Wire.begin(SDA_PIN, SCL_PIN);
    delay(200); // Allow I2C bus to stabilize LONGER before setting clock
    Wire.setClock(400000); // Set I2C clock BEFORE initializing devices
    delay(100); // Additional delay for clock to stabilize
    
    LOG_INFO("I2C initialized on pins SDA=%d, SCL=%d at 400kHz", SDA_PIN, SCL_PIN);
    
    delay(50);
    lcd = new DFRobot_RGBLCD1602(LCD_I2C_ADDR, LCD_LINE_LENGTH, LCD_ROWS, &Wire);
    
    // Verify LCD is present BEFORE calling init() (which may timeout)
    LOG_INFO("Checking if LCD is present at address 0x%02x...", LCD_I2C_ADDR);
    Wire.beginTransmission(LCD_I2C_ADDR);
    byte error = Wire.endTransmission(true);
    
    if (error == 0) {
      LOG_INFO("LCD found at address 0x%02x - initializing display", LCD_I2C_ADDR);
      // Now safe to call init() since device responded
      lcd->init();
      lcd->setBacklight(true);
      lcdAvailable = true;
      LOG_INFO("LCD initialized successfully");
    } else {
      LOG_ERROR("LCD NOT found at address 0x%02x (error=%d) - LCD disabled", LCD_I2C_ADDR, error);
      LOG_WARN("Make sure LCD is properly connected to SDA=%d, SCL=%d", SDA_PIN, SCL_PIN);
      lcdAvailable = false;
    }

    while (readBatteryPercentage() < CRITICAL_BATTERY_LEVEL) {
      LOG_ERROR("Batterie trop faible pour initialiser. Chargez SVP!");
      
      // Display current voltage on the LCD for debugging
      if (lcdAvailable) {
        float voltage = readBatteryVoltage();
        char buffer[16];
        sprintf(buffer, "Batt: %.2fV", voltage);
        lcd->clear();
        lcd->print(buffer);
      }

      // Flash red on all LEDs using the new system logic if possible, 
      // or just manual for this very early boot stage if robot isn't fully ready.
      led_fx_set_all( (millis() % 1000 < 500) ? 255 : 0, 0, 0);
      delay(1000);
    }
    led_fx_off();
    LOG_INFO("Batterie OK pour l'initialisation.");

    preferences.begin(NVS_NAMESPACE, false);
    // Load communication mode from NVS, fallback to DEFAULT_COMM_MODE from config.h
    int savedCommMode = preferences.getInt(NVS_COMM_MODE_KEY, -1);  // -1 = not found in NVS
    robot.activeCommMode = (savedCommMode >= 0) ? (CommunicationMode)savedCommMode : (CommunicationMode)DEFAULT_COMM_MODE;
    bool commModeFromNVS = (savedCommMode >= 0);  // Track source
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
      if (lcdAvailable) {
        setLcdText(robot, "SD Card Error!");
      } else {
        LOG_ERROR("SD Card Error and LCD not available!");
      }
      while (true);
    }
    
    if (robot.activeCommMode == COMM_MODE_XBOX) {
        LOG_INFO("COMM MODE: Xbox Controller (from %s)", commModeFromNVS ? "NVS" : "config DEFAULT_COMM_MODE");
        xboxController.begin();
    } else {
        LOG_INFO("COMM MODE: Serial Only (from %s)", commModeFromNVS ? "NVS" : "config DEFAULT_COMM_MODE");
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

    // Run formal self-test before proceeding
    bool selfTestPassed = runSelfTest(robot);
    if (!selfTestPassed) {
        LOG_WARN("Self-test reported failures - proceeding anyway");
    }

    // The turret servo (tourelle) is attached/detach...
    // to manage power. The direction servo (Servodirection) is attached manually here
    // and remains attached for immediate control.
    Servodirection.attach(PINDIRECTION);
    Servodirection.write(robot.servoNeutralDir);
    
    // Test turret servo at startup to verify it's working
    LOG_INFO("Testing turret servo at neutral position...");
    tourelle.attach();
    tourelle.write(robot.servoNeutralTurret, robot.servoNeutralTurret);
    delay(500);
    tourelle.detach(); // Detach it for power saving in IDLE
    LOG_INFO("Turret servo test complete");

    // LED startup test sequence
    led_fx_startup_test();

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

  // Handle Xbox controller input (non-blocking, thread-safe if it only writes to robot state handled by next mutex)
  if (robot.activeCommMode == COMM_MODE_XBOX) {
    xboxController.processControllers();
  }
  
  // Serial input (non-blocking read)
  checkSerial();

  // Battery monitoring (non-critical data reads)
  updateBatteryStatus(robot);

  // CRITICAL SECTION #1: Safety & Power (Bumper, Battery Critical)
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
    xSemaphoreGive(robotMutex);
  }

  // Sensor updates that don't need the mutex for the hardware part
  sensor_update_task(robot);
  
  // CRITICAL SECTION #2: Sensors, Navigation & Motors
  if(xSemaphoreTake(robotMutex, (TickType_t) MUTEX_WAIT_TICKS) == pdTRUE) {
    // 1. Update Turret
    updateTurret(robot);
    
    // 2. Compass update
    if (robot.currentState != IDLE || (millis() - robot.lastCompassReadTime > COMPASS_READ_INTERVAL_MS)) {
      float heading = getCalibratedHeading(robot);
      if (!std::isnan(heading)) {
        robot.cap = heading;
      }
      robot.lastCompassReadTime = millis();
    }
    robot.currentPitch = getPitch(robot);

    // 3. Laser distance
    if (robot.laserInitialized && vl53->dataReady()) {
      robot.distanceLaser = vl53->readRangeContinuousMillimeters() / MM_PER_CM;
    }

    // 4. Motor control & Telemetry
    updateMotorControl(robot);

    if (millis() - robot.lastReportTime > robot.reportInterval) {
      robot.lastReportTime = millis();
      sendTelemetry(robot);
    }

    // 5. Display & FX Updates (using fresh state)
    displayJokesIfIdle(robot);
    updateLcdDisplay(robot);
    handleLcdAnimations(robot);
    led_fx_update(robot);

    xSemaphoreGive(robotMutex);
  }

  // Calculate loop timing
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

