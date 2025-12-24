#include "config.h"
#include "hardware.h"
#include "comms.h" // For setLcdText

// --- HARDWARE OBJECTS DEFINITIONS ---
MX1508 motorA(AIN1, AIN2);
MX1508 motorB(BIN1, BIN2);
Servo Servodirection;
Tourelle tourelle(PINTOURELLE_H, PINTOURELLE_V);
LSM303 *compass;
DFRobot_RGBLCD1602 *lcd;
VL53L1X *vl53;
Adafruit_NeoPixel pixels(NEOPIXEL_COUNT, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
Preferences preferences;


// --- RTOS & INTERRUPT ---
SemaphoreHandle_t robotMutex = NULL;
volatile bool bumperPressed = false;

void IRAM_ATTR onBumperPress() {
  bumperPressed = true;
}

// --- Tourelle Class Implementation ---
Tourelle::Tourelle(int pinH, int pinV) : servoH_pin(pinH), servoV_pin(pinV) {}

void Tourelle::attach() {
    servoH.attach(servoH_pin);
    servoV.attach(servoV_pin);
}

void Tourelle::detach() {
    servoH.detach();
    servoV.detach();
}

void Tourelle::write(int angleH, int angleV) {
    servoH.write(angleH);
    servoV.write(angleV);
}

int Tourelle::getAngleHorizontal() {
    return servoH.read();
}

int Tourelle::getAngleVertical() {
    return servoV.read();
}


// --- GENERAL ---
void Arret() {
    motorA.motorBrake();
    motorB.motorBrake();
}

void updateBatteryStatus(Robot& robot) {
  int batteryLevel = readBatteryPercentage();

  if (batteryLevel < CRITICAL_BATTERY_LEVEL) {
    if (!robot.batteryIsCritical) {
      robot.batteryIsCritical = true;
      robot.batteryIsLow = true; 
    }
    return;
  }

  if (robot.batteryIsLow || robot.batteryIsCritical) {
    if (batteryLevel >= RECHARGED_THRESHOLD) {
      robot.speedAvg = robot.initialSpeedAvg;
      robot.speedSlow = robot.initialSpeedSlow;
      robot.batteryIsLow = false;
      robot.batteryIsCritical = false;
    }
  } 
  else {
    if (batteryLevel < LOW_BATTERY_THRESHOLD) {
      if (!robot.batteryIsLow) {
        robot.speedAvg = robot.speedSlow; 
        robot.speedSlow = robot.speedSlow / 2; 
        robot.batteryIsLow = true;
      }
    }
  }
}

// --- COMPASS ---
#include <EEPROM.h>

// EEPROM addresses for calibration data
#define EEPROM_MAGIC_NUMBER_ADDR 0
#define EEPROM_COMPASS_OFFSET_ADDR 4
#define EEPROM_COMPASS_CALIBRATED_ADDR 8
#define EEPROM_MAG_MIN_X_ADDR 12
#define EEPROM_MAG_MIN_Y_ADDR 16
#define EEPROM_MAG_MIN_Z_ADDR 20
#define EEPROM_MAG_MAX_X_ADDR 24
#define EEPROM_MAG_MAX_Y_ADDR 28
#define EEPROM_MAG_MAX_Z_ADDR 32
#define EEPROM_MAGIC_VALUE 12345
#define EEPROM_SIZE 64

#define COMPASS_CALIBRATION_DURATION_MS 15000
#define COMPASS_CALIBRATION_VALIDATION_THRESHOLD 32000
#define COMPASS_MIN_INT16 -32768
#define COMPASS_MAX_INT16 32767

// Implementations from compass.h...
void compass_init(Robot& robot) {
    if (DEBUG_MODE) Serial.println("Initialisation du compas...");
    EEPROM.begin(EEPROM_SIZE);
    if (!compass->init()) {
        if (DEBUG_MODE) Serial.println("ERREUR: Impossible d'initialiser le compas!");
        setLcdText(robot, "ERREUR COMPAS");
        robot.compassInitialized = false;
    } else {
        if (DEBUG_MODE) Serial.println("Compas initialise avec succes!");
        robot.compassInitialized = true;
        compass->enableDefault();
        loadCompassCalibration(robot);
    }
}

float getCalibratedHeading(Robot& robot) {
    if (!robot.compassInitialized) return 0.0;
    
    compass->read();
    if (compass->m.x == 0 && compass->m.y == 0 && compass->m.z == 0) {
        if (DEBUG_MODE) Serial.println("ATTENTION: Valeurs compas nulles dans getCalibratedHeading ()");
        return 0.0;
    }

    float heading;
    if (robot.compassCalibrated) {
        // Correct for hard-iron distortion
        float corrected_x = compass->m.x - (robot.magMin.x + robot.magMax.x) / 2.0;
        float corrected_y = compass->m.y - (robot.magMin.y + robot.magMax.y) / 2.0;

        // Correct for soft-iron distortion
        float avg_delta_x = (robot.magMax.x - robot.magMin.x) / 2.0;
        float avg_delta_y = (robot.magMax.y - robot.magMin.y) / 2.0;
        
        if (avg_delta_x == 0 || avg_delta_y == 0) {
            heading = calculateHeading(*compass);
        } else {
            float avg_delta = (avg_delta_x + avg_delta_y) / 2.0;
            float scaled_x = corrected_x * (avg_delta / avg_delta_x);
            float scaled_y = corrected_y * (avg_delta / avg_delta_y);
            heading = calculateHeading(scaled_y, scaled_x);
        }
    } else {
        heading = calculateHeading(*compass);
    }

    // Apply inversion and fine-tuning offset
    if (robot.compassInverted) {
        heading += 180.0;
    }
    heading += robot.compassOffset;

    // Normalize heading to 0-359.9 degrees
    while (heading < 0.0) heading += 360.0;
    while (heading >= 360.0) heading -= 360.0;
    
    return heading;
}

void calibrateCompass(Robot& robot) {
    static unsigned long startTime = 0;
    static bool calibrationStarted = false;

    if (!calibrationStarted) {
        Serial.println("=== CALIBRATION DU COMPAS (360 DEGRES) ===");
        robot.magMin = {COMPASS_MAX_INT16, COMPASS_MAX_INT16, COMPASS_MAX_INT16};
        robot.magMax = {COMPASS_MIN_INT16, COMPASS_MIN_INT16, COMPASS_MIN_INT16};
        startTime = millis();
        calibrationStarted = true;
        setLcdText(robot, "Calib 360...");
    }

    if (millis() - startTime < COMPASS_CALIBRATION_DURATION_MS) {
        compass->read();
        robot.magMin.x = min(robot.magMin.x, compass->m.x);
        robot.magMin.y = min(robot.magMin.y, compass->m.y);
        robot.magMin.z = min(robot.magMin.z, compass->m.z);
        robot.magMax.x = max(robot.magMax.x, compass->m.x);
        robot.magMax.y = max(robot.magMax.y, compass->m.y);
        robot.magMax.z = max(robot.magMax.z, compass->m.z);
    } else {
        Serial.println("=== CALIBRATION TERMINEE ===");
        if (robot.magMin.x < COMPASS_CALIBRATION_VALIDATION_THRESHOLD && robot.magMax.x > -COMPASS_CALIBRATION_VALIDATION_THRESHOLD) {
            robot.compassCalibrated = true;
            Serial.println("✅ Calibration REUSSIE.");
            setLcdText(robot, "Calib. OK");
        } else {
            robot.compassCalibrated = false;
            Serial.println("⚠️ Calibration DEFAILLANTE.");
            setLcdText(robot, "Calib. ECHEC");
        }
        saveCompassCalibration(robot);
        calibrationStarted = false;
        robot.currentState = IDLE; // Return to IDLE state
    }
}

void saveCompassCalibration(const Robot& robot) {
    if (DEBUG_MODE) Serial.println("Sauvegarde de la calibration en EEPROM...");
    EEPROM.put(EEPROM_MAGIC_NUMBER_ADDR, EEPROM_MAGIC_VALUE);
    EEPROM.put(EEPROM_COMPASS_OFFSET_ADDR, robot.compassOffset);
    EEPROM.put(EEPROM_COMPASS_CALIBRATED_ADDR, robot.compassCalibrated);
    EEPROM.put(EEPROM_MAG_MIN_X_ADDR, robot.magMin.x);
    EEPROM.put(EEPROM_MAG_MIN_Y_ADDR, robot.magMin.y);
    EEPROM.put(EEPROM_MAG_MIN_Z_ADDR, robot.magMin.z);
    EEPROM.put(EEPROM_MAG_MAX_X_ADDR, robot.magMax.x);
    EEPROM.put(EEPROM_MAG_MAX_Y_ADDR, robot.magMax.y);
    EEPROM.put(EEPROM_MAG_MAX_Z_ADDR, robot.magMax.z);
    if (!EEPROM.commit()) {
        if (DEBUG_MODE) Serial.println("ERREUR: Impossible de sauvegarder la calibration en EEPROM!");
    } else {
        if (DEBUG_MODE) Serial.println("Calibration sauvegardee en EEPROM.");
    }
}

void loadCompassCalibration(Robot& robot) {
    if (isEEPROMDataValid()) {
        EEPROM.get(EEPROM_COMPASS_OFFSET_ADDR, robot.compassOffset);
        EEPROM.get(EEPROM_COMPASS_CALIBRATED_ADDR, robot.compassCalibrated);
        EEPROM.get(EEPROM_MAG_MIN_X_ADDR, robot.magMin.x);
        EEPROM.get(EEPROM_MAG_MIN_Y_ADDR, robot.magMin.y);
        EEPROM.get(EEPROM_MAG_MIN_Z_ADDR, robot.magMin.z);
        EEPROM.get(EEPROM_MAG_MAX_X_ADDR, robot.magMax.x);
        EEPROM.get(EEPROM_MAG_MAX_Y_ADDR, robot.magMax.y);
        EEPROM.get(EEPROM_MAG_MAX_Z_ADDR, robot.magMax.z);
        if (DEBUG_MODE) Serial.println("Calibration compas chargee depuis l'EEPROM.");
    } else {
        if (DEBUG_MODE) Serial.println("Aucune calibration valide trouvee en EEPROM.");
        robot.compassCalibrated = false;
    }
}

bool isEEPROMDataValid() {
    int magicValue = 0;
    EEPROM.get(EEPROM_MAGIC_NUMBER_ADDR, magicValue);
    return (magicValue == EEPROM_MAGIC_VALUE);
}

float calculateHeading(float y, float x) {
  float heading = (atan2(y, x) * 180) / PI;
  if (heading < 0) {
    heading += 360;
  }
  return heading;
}

float calculateHeading(const LSM303& compass) {
  return calculateHeading(compass.m.y, compass.m.x);
}

float getPitch(Robot& robot) {
    // Read accelerometer data
    compass->readAcc();

    // Calculate pitch using atan2 for robustness
    float pitch = atan2(compass->a.y, -compass->a.z) * 180.0 / PI;

    return pitch;
}

bool detectImpactOrStall(Robot& robot) {
    // Detects sudden physical impacts or prolonged stall conditions using accelerometer data.
    static float lastAccelX = 0;
    static float lastAccelY = 0;
    
    compass->readAcc();
    float currentX = compass->a.x;
    float currentY = compass->a.y; 
    
    float jerk = abs(currentX - lastAccelX) + abs(currentY - lastAccelY);
    lastAccelX = currentX;
    lastAccelY = currentY;

    // If the jerk is huge, it's an immediate impact
    if (jerk > 18000) { // Value needs calibration for specific sensor sensitivity
        if(DEBUG_MODE) Serial.println("IMU: IMPACT DETECTED!");
        return true;
    }
    return false;
}

// --- LED FX ---
// Implementations from led_fx.h...
void led_fx_init() {
    pixels.begin();
    pixels.clear();
    pixels.show();
}

// Helper to set color on the 3 visible LEDs only (indices 1, 2, 3)
void led_fx_set_visible_only(uint32_t color) {
    for(int i = 1; i < NEOPIXEL_COUNT; i++) {
        pixels.setPixelColor(i, color);
    }
}

void led_fx_update(const Robot& robot) {
    static unsigned long last_update = 0;
    const int update_interval = 50; // ms

    if (millis() - last_update < update_interval) {
        return; // Update LEDs at a fixed interval
    }
    last_update = millis();

    pixels.clear();

    if (robot.batteryIsCritical) {
        uint32_t color = (millis() % 500 < 250) ? pixels.Color(255, 0, 0) : 0;
        pixels.setPixelColor(0, color);
        led_fx_set_visible_only(color);
        pixels.show();
        return;
    }
    
    if (robot.currentState == CLIFF_DETECTED) {
        uint32_t color = (millis() % 1000 < 500) ? pixels.Color(255, 165, 0) : 0;
        led_fx_set_visible_only(color);
        pixels.show();
        return;
    }

    if (robot.currentState == OBSTACLE_AVOIDANCE) {
        static int eye_pos = 1;
        static int eye_dir = 1;
        pixels.setPixelColor(eye_pos, pixels.Color(255, 0, 0));
        
        eye_pos += eye_dir;
        if (eye_pos > (NEOPIXEL_COUNT - 1) || eye_pos < 1) {
            eye_dir *= -1;
            eye_pos += eye_dir * 2;
        }
        pixels.show();
        return;
    }

    if (robot.currentState == SENTRY_MODE) {
        if(robot.sentryState == SENTRY_TRACKING) {
            uint32_t color = (millis() % 200 < 100) ? pixels.Color(255, 0, 0) : pixels.Color(255, 255, 0);
            led_fx_set_visible_only(color);
        } else {
            static int eye_pos = 1;
            static int eye_dir = 1;
            pixels.setPixelColor(eye_pos, pixels.Color(0, 0, 255));
            
            eye_pos += eye_dir;
            if (eye_pos > (NEOPIXEL_COUNT - 1) || eye_pos < 1) {
                eye_dir *= -1;
                eye_pos += eye_dir * 2;
            }
        }
        pixels.show();
        return;
    }
    
    switch (robot.currentState) {
        case IDLE: {
            float breath = (sin(millis() / 2000.0 * 2 * PI) + 1) / 2;
            led_fx_set_visible_only(pixels.Color(0, 0, 128 * breath));
            break;
        }
        case MOVING_FORWARD:
        case MANUAL_FORWARD:
        case FOLLOW_HEADING:
        case SMART_AVOIDANCE:
            led_fx_set_visible_only(pixels.Color(0, 255, 0)); // Green
            break;
        case MOVING_BACKWARD:
        case MANUAL_BACKWARD:
            led_fx_set_visible_only(pixels.Color(255, 100, 0)); // Orange-ish
            break;
        case TURNING_LEFT:
        case MANUAL_TURNING_LEFT:
            pixels.setPixelColor(3, pixels.Color(255, 255, 0)); // Yellow
            break;
        case TURNING_RIGHT:
        case MANUAL_TURNING_RIGHT:
            pixels.setPixelColor(1, pixels.Color(255, 255, 0)); // Yellow
            break;
        case CALIBRATING_COMPASS:
            for(int i=1; i<NEOPIXEL_COUNT; i++) {
                pixels.setPixelColor(i, pixels.gamma32(pixels.ColorHSV(((i * 256 / 3) + (millis()/10)) & 255, 255, 255)));
            }
            break;
        default:
            if (robot.batteryIsLow) {
                uint32_t color = (millis() % 2000 < 1000) ? pixels.Color(150, 150, 0) : pixels.Color(30, 30, 0);
                led_fx_set_visible_only(color);
            }
            break;
    }
    pixels.show();
}

// --- SD UTILS ---
// Implementations from sd_utils.cpp...
bool setupSDCard() {
    if (DEBUG_MODE) Serial.print(F("Initializing SD card..."));
    SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS_PIN);
    if (!SD.begin(SD_CS_PIN)) {
        if (DEBUG_MODE) Serial.println(F("SD Card initialization failed!"));
        return false;
    }
    if (DEBUG_MODE) Serial.println(F("SD Card initialized."));
    return true;
}

void getRandomJokeFromSD(Robot& robot, const char* filename, char* buffer, size_t bufferSize) {
    if (!robot.sdCardReady) {
        if (DEBUG_MODE) {
            Serial.println(F("SD Card not ready for getRandomJokeFromSD."));
        }
        strncpy(buffer, "Error: SD Card not ready!", bufferSize - 1);
        buffer[bufferSize - 1] = '\0';
        return;
    }
    File jokesFile = SD.open(filename);
    if (!jokesFile) {
        if (DEBUG_MODE) {
            Serial.print(F("Error opening "));
            Serial.println(filename);
        }
        strncpy(buffer, "Error: SD file not found!", bufferSize - 1);
        buffer[bufferSize - 1] = '\0';
        return;
    }

    int numLines = 0;
    while (jokesFile.available()) {
        if (jokesFile.read() == '\n') {
            numLines++;
        }
    }
    jokesFile.seek(0);

    if (numLines == 0) {
        jokesFile.close();
        strncpy(buffer, "Error: No jokes found!", bufferSize - 1);
        buffer[bufferSize - 1] = '\0';
        return;
    }

    long randomIndex = random(numLines);

    for (long i = 0; i < randomIndex; i++) {
        while(jokesFile.available() && jokesFile.read() != '\n');
    }

    if (jokesFile.available()) {
        size_t bytesRead = jokesFile.readBytesUntil('\n', buffer, bufferSize - 1);
        buffer[bytesRead] = '\0';
    } else {
        buffer[0] = '\0';
    }

    jokesFile.close();
}

bool loadConfig(Robot& robot) {
    if (!robot.sdCardReady) {
        if (DEBUG_MODE) {
            Serial.println(F("SD Card not ready for loadConfig."));
        }
        return false;
    }

    File configFile = SD.open("/config.txt");
    if (!configFile) {
        if (DEBUG_MODE) {
            Serial.println(F("Config file '/config.txt' not found! Using defaults."));
        }
        return false;
    }

    if (DEBUG_MODE) {
        Serial.println(F("Loading config from /config.txt..."));
    }

    String line;
    while (configFile.available()) {
        line = configFile.readStringUntil('\n');
        line.trim();

        if (line.isEmpty() || line.startsWith("#")) {
            continue;
        }

        int equalsIndex = line.indexOf('=');
        if (equalsIndex == -1) {
            if (DEBUG_MODE) {
                Serial.print(F("Invalid config line: "));
                Serial.println(line);
            }
            continue;
        }

        String key = line.substring(0, equalsIndex);
        String value = line.substring(equalsIndex + 1);
        key.trim();
        value.trim();

        if (key.equalsIgnoreCase("VITESSE_MOYENNE")) {
            robot.speedAvg = value.toInt();
        } else if (key.equalsIgnoreCase("VITESSE_LENTE")) {
            robot.speedSlow = value.toInt();
        } else if (key.equalsIgnoreCase("VITESSE_ROTATION")) {
            robot.speedRotation = value.toInt();
        } else if (key.equalsIgnoreCase("TOLERANCE_VIRAGE")) {
            robot.turnTolerance = value.toFloat();
        } else if (key.equalsIgnoreCase("KP_HEADING")) {
            robot.KpHeading = value.toFloat();
        } else if (key.equalsIgnoreCase("VITESSE_ROTATION_MAX")) {
            robot.speedRotationMax = value.toInt();
        } else if (key.equalsIgnoreCase("CALIBRATION_MOTEUR_B")) {
            robot.motorBCalibration = value.toFloat();
        } else if (key.equalsIgnoreCase("SEUIL_BASCULE_DIRECTION")) {
            robot.pivotAngleThreshold = value.toFloat();
        } else if (key.equalsIgnoreCase("MIN_SPEED_TO_MOVE")) {
            robot.minSpeedToMove = value.toInt();
        } else if (key.equalsIgnoreCase("NEUTRE_DIRECTION")) {
            robot.servoNeutralDir = value.toInt();
        } else if (key.equalsIgnoreCase("NEUTRE_TOURELLE")) {
            robot.servoNeutralTurret = value.toInt();
        } else if (key.equalsIgnoreCase("SERVO_DIR_MIN")) {
            robot.servoDirMin = value.toInt();
        } else if (key.equalsIgnoreCase("SERVO_DIR_MAX")) {
            robot.servoDirMax = value.toInt();
        } else if (key.equalsIgnoreCase("ANGLE_TETE_BASSE")) {
            robot.servoAngleHeadDown = value.toInt();
        } else if (key.equalsIgnoreCase("ANGLE_SOL")) {
            robot.servoAngleGround = value.toInt();
        } else if (key.equalsIgnoreCase("ACCEL_RATE")) {
            robot.accelRate = value.toFloat();
        } else if (key.equalsIgnoreCase("DIFF_STRENGTH")) {
            robot.diffStrength = value.toFloat();
        } else if (key.equalsIgnoreCase("FWD_DIFF_COEFF")) {
            robot.fwdDiffCoeff = value.toFloat();
        } else if (key.equalsIgnoreCase("AVOID_BACKUP_DURATION_MS")) {
            robot.avoidBackupDuration = value.toInt();
        } else if (key.equalsIgnoreCase("MIN_DIST_FOR_VALID_PATH")) {
            robot.minDistForValidPath = value.toInt();
        } else if (key.equalsIgnoreCase("TURRET_MOVE_TIME_MS")) {
            robot.turretMoveTime = value.toInt();
        } else if (key.equalsIgnoreCase("SEUIL_VIDE")) {
            robot.seuilVide = value.toInt();
        } else if (key.equalsIgnoreCase("VL53L1X_TIMING_BUDGET_US")) {
            robot.laserTimingBudget = value.toInt();
        } else if (key.equalsIgnoreCase("VL53L1X_INTER_MEASUREMENT_PERIOD_MS")) {
            robot.laserInterMeasurementPeriod = value.toInt();
        } else if (key.equalsIgnoreCase("INITIAL_AUTONOMOUS_DELAY_MS")) {
            robot.initialAutonomousDelay = value.toInt();
        }
    }

    configFile.close();
    if (DEBUG_MODE) {
        Serial.println(F("Config loaded."));
    }
    return true;
}


// --- SENSOR TASK ---
// Implementations from sensor_task.h...
void sensor_init() {
  pinMode(TRIGGER, OUTPUT);
  pinMode(ECHO, INPUT);
}

void sensor_update_task(Robot& robot) {
  static unsigned long last_ping_time = 0;
  unsigned long current_time = millis();

  if (current_time - last_ping_time > ULTRASONIC_PING_INTERVAL_MS) {
    last_ping_time = current_time;

    digitalWrite(TRIGGER, LOW);
    delayMicroseconds(ULTRASONIC_TRIGGER_PULSE_LOW_US);
    digitalWrite(TRIGGER, HIGH);
    delayMicroseconds(ULTRASONIC_TRIGGER_PULSE_HIGH_US);
    digitalWrite(TRIGGER, LOW);

    unsigned long duration = pulseIn(ECHO, HIGH, ULTRASONIC_PULSE_TIMEOUT_US);

    if (duration > 0) {
      robot.dusm = duration / ULTRASONIC_DURATION_TO_CM_DIVISOR;
    } else {
      robot.dusm = ULTRASONIC_ERROR_VALUE;
    }
  }
}


// --- SUPPORT ---
// Implementations from support.h...
void PhareAllume() {
  digitalWrite(PIN_PHARE, HIGH);
}

void PhareEteint() {
  digitalWrite(PIN_PHARE, LOW);
}

float readBatteryVoltage() {
    const int BATTERY_READINGS_COUNT = 10;
    static float batteryReadings[BATTERY_READINGS_COUNT] = {0};
    static int readingsIndex = 0;
    static bool readingsFilled = false;
    static float sum = 0;

    sum -= batteryReadings[readingsIndex];

    int rawReading = analogRead(VBAT);
    float currentVoltage = (rawReading * 3.3 / 4095.0) * ((18.0 + 10.0) / 10.0);
    batteryReadings[readingsIndex] = currentVoltage;
    sum += currentVoltage;

    readingsIndex++;
    if (readingsIndex >= BATTERY_READINGS_COUNT) {
        readingsIndex = 0;
        readingsFilled = true;
    }

    int count = readingsFilled ? BATTERY_READINGS_COUNT : readingsIndex;
    if (count == 0) return 0;
    
    return sum / count;
}

int readBatteryPercentage() {
  float batteryVoltage = readBatteryVoltage();
  float maxVoltage, minVoltage;

#if SELECTED_BATTERY_TYPE == BATTERY_TYPE_LIPO
  maxVoltage = LIPO_MAX_VOLTAGE;
  minVoltage = LIPO_MIN_VOLTAGE;
#elif SELECTED_BATTERY_TYPE == BATTERY_TYPE_NIMH
  maxVoltage = NIMH_MAX_VOLTAGE;
  minVoltage = NIMH_MIN_VOLTAGE;
#else
  maxVoltage = NIMH_MAX_VOLTAGE;
  minVoltage = NIMH_MIN_VOLTAGE;
#endif

  if (maxVoltage <= minVoltage) {
    return 0;
  }

  float percentage = ((batteryVoltage - minVoltage) / (maxVoltage - minVoltage)) * 100.0;
  
  return constrain(static_cast<int>(percentage), 0, 100);
}


// --- TURRET CONTROL ---
// Implementations from turret_control.cpp and turret_control.h
unsigned long lastTurretUpdate = 0;
int turretPanAngle = 90;
int turretTiltAngle = 90;
int turretScanDirection = 1;

void stabilizeTurretTilt(Robot& robot) {
    float pitch = getPitch(robot);
    int targetTilt = 90 - (int)(pitch * 0.8); // TURRET_TILT_STABILITY_FACTOR
    turretTiltAngle = constrain(targetTilt, 70, 110);
}

void updateTurret(Robot& robot, bool isMovingForward) {
    if (millis() - lastTurretUpdate > 30) { // TURRET_UPDATE_INTERVAL
        lastTurretUpdate = millis();

        stabilizeTurretTilt(robot);

        switch (robot.currentState) {
            case MOVING_FORWARD:
            case FOLLOW_HEADING:
            case MAINTAIN_HEADING:
                turretPanAngle += (2 * turretScanDirection); // PREDICTIVE_SCAN_STEP
                if (turretPanAngle >= 120 || turretPanAngle <= 60) { // PREDICTIVE_SCAN_MAX_ANGLE, PREDICTIVE_SCAN_MIN_ANGLE
                    turretScanDirection *= -1;
                }
                break;

            case OBSTACLE_AVOIDANCE:
                turretPanAngle += (5 * turretScanDirection); // WIDE_SCAN_STEP
                if (turretPanAngle >= 170 || turretPanAngle <= 10) { // WIDE_SCAN_MAX_ANGLE, WIDE_SCAN_MIN_ANGLE
                    turretScanDirection *= -1;
                }
                break;
            
            case TURNING_LEFT:
            case TURNING_RIGHT:
                // "Look-Ahead" is handled by handleFollowHeadingState, which calls syncTurretWithSteering.
                break;

            case IDLE:
            default:
                if(turretPanAngle > 90) turretPanAngle--;
                else if (turretPanAngle < 90) turretPanAngle++;
                if(turretTiltAngle > 95) turretTiltAngle--;
                else if (turretTiltAngle < 95) turretTiltAngle++;
                break;
        }
        
        tourelle.write(constrain(turretPanAngle, 10, 170), turretTiltAngle);
    }
}

void syncTurretWithSteering(int steeringAngle) {
    int headAngle = 90 + (steeringAngle - 90) * 1.5;
    turretPanAngle = constrain(headAngle, 10, 170); // WIDE_SCAN_MIN_ANGLE, WIDE_SCAN_MAX_ANGLE
    tourelle.write(turretPanAngle, turretTiltAngle);
}

int findClearestPath(Robot& robot) {
    int bestAngle = -1;
    float maxScore = -9999;
    
    for (int angle = SCAN_H_START_ANGLE; angle <= SCAN_H_END_ANGLE; angle += SCAN_H_STEP) {
        tourelle.write(angle, robot.servoNeutralTurret);
        delay(robot.turretScanDelay); 
        
        int distLaser;
        if (robot.laserInitialized) {
            distLaser = vl53->readRangeContinuousMillimeters() / 10;
            if (distLaser == 65535 / 10) {
                distLaser = robot.maxUltrasonicDistance;
            }
        } else {
            if (DEBUG_MODE) Serial.println("WARNING: Laser not initialized, findClearestPath relying solely on ultrasonic.");
            distLaser = robot.maxUltrasonicDistance;
        }
        
        int distUS = robot.dusm;
        
        int effectiveDist = (distUS > 0 && distUS < distLaser) ? distUS : distLaser;
        if (effectiveDist <= 0) effectiveDist = robot.maxUltrasonicDistance;

        float angleDeviation = abs(90 - angle);
        float score = effectiveDist - (angleDeviation * robot.anglePenaltyFactor);

        if (effectiveDist > robot.minDistForValidPath && score > maxScore) {
            maxScore = score;
            bestAngle = angle;
        }
    }

    return bestAngle;
}

int setAckermannAngle(Robot& robot, int angleError, int speed) {
    int limit = 10; //getDynamicMaxSteering(robot, speed);
    
    int clampedError = constrain(angleError, -limit, limit);

    int servoAngle = map(clampedError, -robot.pivotAngleThreshold, robot.pivotAngleThreshold, robot.servoDirMin, robot.servoDirMax);
    servoAngle = constrain(servoAngle, robot.servoDirMin, robot.servoDirMax);
    Servodirection.write(servoAngle);
    return servoAngle;
}
