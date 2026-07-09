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

// --- Task Prototypes ---
void controlTask(void* pvParameters);
void uiTelemetryTask(void* pvParameters);

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
    Wire.setTimeOut(10); // Set global Wire timeout (10ms) to prevent I2C hangs
    delay(200); // Allow I2C bus to stabilize LONGER before setting clock
    Wire.setClock(400000); // Set I2C clock BEFORE initializing devices
    delay(100); // Additional delay for clock to stabilize
    
    LOG_INFO("I2C initialized on pins SDA=%d, SCL=%d at 400kHz with 10ms timeout", SDA_PIN, SCL_PIN);
    
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
        snprintf(buffer, sizeof(buffer), "Batt: %.2fV", voltage);
        lcd->clear();
        lcd->print(buffer);
      }

      // Flash red on all LEDs using the new system logic if possible
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
    i2cMutex = xSemaphoreCreateMutex();
    delay(500);

    compass = new LSM303();
    vl53 = new VL53L1X();

    if (setupSDCard()) {
      robot.sdCardReady = true;
      loadConfig(robot); 
      robot.initialSpeedAvg = robot.speedAvg;
      robot.initialSpeedSlow = robot.speedSlow;
    } else {
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

    // Create FreeRTOS Tasks
    xTaskCreatePinnedToCore(
        controlTask,
        "ControlTask",
        8192,
        NULL,
        5, // High priority
        NULL,
        1  // Core 1
    );

    xTaskCreatePinnedToCore(
        uiTelemetryTask,
        "UiTelemetryTask",
        8192,
        NULL,
        1, // Low priority
        NULL,
        0  // Core 0
    );
}

// --- MAIN LOOP (IDLE SLEEP) ---
void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}

// --- High Priority Control & Safety Task (Core 1) ---
void controlTask(void* pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(10); // 100Hz (10ms)

    while (true) {
        if (xSemaphoreTake(robotMutex, portMAX_DELAY) == pdTRUE) {
            // 1. Process controller input if Xbox mode is active
            if (robot.activeCommMode == COMM_MODE_XBOX) {
                xboxController.processControllers();
            }

            // 2. Safety override: check hardware bumper from ISR
            if (bumperPressed) {
                bumperPressed = false;
                if (robot.currentState != EMERGENCY_EVASION && robot.currentState != MANUAL_COMMAND_MODE) {
                    trigger_rumble(100, 0, 255);
                    robot.stateBeforeEvasion = robot.currentState;
                    changeState(robot, EMERGENCY_EVASION, AVOID_IDLE);
                }
            }

            // 3. Safety override: critical battery check
            if (robot.batteryIsCritical) {
                Arret();
                // Skip sensor reading and motor regulation to stay parked safely
            } else {
                // 4. Update non-blocking sensors (ultrasonic)
                sensor_update_task(robot);

                // 5. Update Turret stabilization/movements
                updateTurret(robot);

                // 6. Update I2C sensors with dedicated mutex & short timeouts
                if (robot.currentState != IDLE || (millis() - robot.lastCompassReadTime > COMPASS_READ_INTERVAL_MS)) {
                    float heading = getCalibratedHeading(robot);
                    if (!std::isnan(heading)) {
                        robot.cap = heading;
                    }
                    robot.lastCompassReadTime = millis();
                }
                robot.currentPitch = getPitch(robot);

                if (robot.laserInitialized) {
                    bool ready = false;
                    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
                        ready = vl53->dataReady();
                        xSemaphoreGive(i2cMutex);
                    }
                    if (ready) {
                        int dist = -1;
                        if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
                            dist = vl53->readRangeContinuousMillimeters();
                            if (vl53->timeoutOccurred()) {
                                LOG_WARN("Laser read timeout");
                            }
                            xSemaphoreGive(i2cMutex);
                        }
                        if (dist > 0) {
                            robot.distanceLaser = dist / MM_PER_CM;
                        }
                    }
                }

                // 7. Motor control regulation & state machine step
                updateMotorControl(robot);
            }

            xSemaphoreGive(robotMutex);
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// --- Low Priority UI & Telemetry Task (Core 0) ---
void uiTelemetryTask(void* pvParameters) {
    const TickType_t xFrequency = pdMS_TO_TICKS(100); // 10Hz (100ms)

    while (true) {
        // 1. Check serial input (takes robotMutex internally during parsing)
        checkSerial();

        // 2. Battery monitoring & LCD updates
        if (xSemaphoreTake(robotMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
            updateBatteryStatus(robot);

            // Check if we need to send telemetry (every 2 seconds)
            bool timeToReport = (millis() - robot.lastReportTime > robot.reportInterval);
            TelemetryData tData;
            if (timeToReport) {
                robot.lastReportTime = millis();
                tData.currentState = robot.currentState;
                tData.cap = robot.cap;
                tData.dusm = robot.dusm;
                tData.distanceLaser = robot.distanceLaser;
                tData.batteryPercentage = readBatteryPercentage();
                tData.targetSpeed = robot.targetSpeed;
            }

            if (robot.batteryIsCritical) {
                setLcdText(robot, "Batterie Vide!");
            } else {
                displayJokesIfIdle(robot);
                updateLcdDisplay(robot);
            }

            handleLcdAnimations(robot);
            led_fx_update(robot);

            xSemaphoreGive(robotMutex);

            // 3. Print Telemetry (executed outside robotMutex to avoid blocking high-priority tasks)
            if (timeToReport) {
                sendTelemetry(tData);
            }
        }

        // Nourrir le watchdog sur le Cœur 0
        vTaskDelay(xFrequency);
    }
}
