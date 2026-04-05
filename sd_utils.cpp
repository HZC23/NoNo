#include "hardware.h"
#include "robot.h"
#include "logger.h"

// Define the global SdFat object
SdFat sd;

bool setupSDCard() {
    LOG_DEBUG("Initializing SD card...");
    SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS_PIN);
    if (!sd.begin(SD_CS_PIN)) {
        LOG_ERROR("SD Card initialization failed!");
        return false;
    }
    LOG_INFO("SD Card initialized.");
    return true;
}

void getRandomJokeFromSD(Robot& robot, const char* filename, char* buffer, size_t bufferSize) {
    if (!robot.sdCardReady) {
        LOG_WARN("SD Card not ready for getRandomJokeFromSD.");
        strncpy(buffer, "Erreur: Carte SD non prete!", bufferSize - 1);
        buffer[bufferSize - 1] = '\0';
        return;
    }
    auto jokesFile = sd.open(filename, O_RDONLY);
    if (!jokesFile) {
        LOG_ERROR("Error opening %s", filename);
        strncpy(buffer, "Erreur: Fichier blagues non trouve!", bufferSize - 1);
        buffer[bufferSize - 1] = '\0';
        return;
    }

    int numLines = 0;
    int lastChar = 0;
    while (jokesFile.available()) {
        lastChar = jokesFile.read();
        if (lastChar == '\n') {
            numLines++;
        }
    }
    // If file didn't end with newline, count the last line
    if (lastChar != '\n' && lastChar != 0) {
        numLines++;
    }
    jokesFile.seekSet(0);

    if (numLines == 0) {
        jokesFile.close();
        LOG_WARN("No jokes found in %s", filename);
        strncpy(buffer, "Pas de blagues trouvees!", bufferSize - 1);
        buffer[bufferSize - 1] = '\0';
        return;
    }

    long randomIndex = random(numLines);

    for (long i = 0; i < randomIndex; i++) {
        while(jokesFile.available() and jokesFile.read() != '\n');
    }

    if (jokesFile.available()) {
        // Manual implementation of readBytesUntil
        size_t bytesRead = 0;
        while (jokesFile.available() && bytesRead < bufferSize - 1) {
            char c = jokesFile.read();
            if (c == '\n') {
                break;
            }
            if (c == '\r') {
                // Skip carriage return from Windows CRLF
                continue;
            }
            buffer[bytesRead++] = c;
        }
        buffer[bytesRead] = '\0';
    } else { 
        buffer[0] = '\0'; // Should not happen if numLines > 0
    }

    jokesFile.close();
}

bool loadConfig(Robot& robot) {
    if (!robot.sdCardReady) {
        LOG_WARN("SD Card not ready for loadConfig.");
        return false;
    }

    auto configFile = sd.open("/config.txt", O_RDONLY);
    if (!configFile) {
        LOG_INFO("Config file '/config.txt' not found! Using defaults.");
        return false;
    }

    LOG_INFO("Loading config from /config.txt...");

    while (configFile.available()) {
        // Manual implementation of readStringUntil
        String line = "";
        while (configFile.available()) {
            char c = configFile.read();
            if (c == '\n' || c == '\r') {
                if (c == '\r' && configFile.peek() == '\n') {
                    configFile.read(); // consume '\n'
                }
                break;
            }
            line += c;
        }
        line.trim();

        if (line.isEmpty() || line.startsWith("#")) {
            continue;
        }

        int equalsIndex = line.indexOf('=');
        if (equalsIndex == -1) {
            LOG_WARN("Invalid config line: %s", line.c_str());
            continue;
        }

        String key = line.substring(0, equalsIndex);
        String value = line.substring(equalsIndex + 1);
        key.trim();
        value.trim();

        // Skip empty values
        if (value.length() == 0) {
            LOG_WARN("Empty config value for key: %s", key.c_str());
            continue;
        }

        if (key.equalsIgnoreCase("VITESSE_MOYENNE")) {
            int parsed = value.toInt();
            if (parsed == 0 && value != "0") {
                LOG_WARN("Invalid VITESSE_MOYENNE value: %s (parsed as 0)", value.c_str());
            } else {
                robot.speedAvg = parsed;
            }
        } else if (key.equalsIgnoreCase("VITESSE_LENTE")) {
            int parsed = value.toInt();
            if (parsed == 0 && value != "0") {
                LOG_WARN("Invalid VITESSE_LENTE value: %s (parsed as 0)", value.c_str());
            } else {
                robot.speedSlow = parsed;
            }
        } else if (key.equalsIgnoreCase("VITESSE_ROTATION")) {
            int parsed = value.toInt();
            if (parsed == 0 && value != "0") {
                LOG_WARN("Invalid VITESSE_ROTATION value: %s (parsed as 0)", value.c_str());
            } else {
                robot.speedRotation = parsed;
            }
        } else if (key.equalsIgnoreCase("TOLERANCE_VIRAGE")) {
            float parsed = value.toFloat();
            if (parsed == 0.0 && !value.startsWith("0")) {
                LOG_WARN("Invalid TOLERANCE_VIRAGE value: %s (parsed as 0.0)", value.c_str());
            } else {
                robot.turnTolerance = parsed;
            }
        } else if (key.equalsIgnoreCase("KP_HEADING")) {
            float parsed = value.toFloat();
            if (parsed == 0.0 && !value.startsWith("0")) {
                LOG_WARN("Invalid KP_HEADING value: %s (parsed as 0.0)", value.c_str());
            } else {
                robot.KpHeading = parsed;
            }
        } else if (key.equalsIgnoreCase("VITESSE_ROTATION_MAX")) {
            int parsed = value.toInt();
            if (parsed == 0 && value != "0") {
                LOG_WARN("Invalid VITESSE_ROTATION_MAX value: %s (parsed as 0)", value.c_str());
            } else {
                robot.speedRotationMax = parsed;
            }
        } else if (key.equalsIgnoreCase("CALIBRATION_MOTEUR_B")) {
            float parsed = value.toFloat();
            if (parsed == 0.0 && !value.startsWith("0")) {
                LOG_WARN("Invalid CALIBRATION_MOTEUR_B value: %s (parsed as 0.0)", value.c_str());
            } else {
                robot.motorBCalibration = parsed;
            }
        } else if (key.equalsIgnoreCase("SEUIL_BASCULE_DIRECTION")) {
            float parsed = value.toFloat();
            if (parsed == 0.0 && !value.startsWith("0")) {
                LOG_WARN("Invalid SEUIL_BASCULE_DIRECTION value: %s (parsed as 0.0)", value.c_str());
            } else {
                robot.pivotAngleThreshold = parsed;
            }
        } else if (key.equalsIgnoreCase("MIN_SPEED_TO_MOVE")) {
            int parsed = value.toInt();
            if (parsed == 0 && value != "0") {
                LOG_WARN("Invalid MIN_SPEED_TO_MOVE value: %s (parsed as 0)", value.c_str());
            } else {
                robot.minSpeedToMove = parsed;
            }
        } else if (key.equalsIgnoreCase("NEUTRE_DIRECTION")) {
            int parsed = value.toInt();
            if (parsed == 0 && value != "0") {
                LOG_WARN("Invalid NEUTRE_DIRECTION value: %s (parsed as 0)", value.c_str());
            } else {
                robot.servoNeutralDir = parsed;
            }
        } else if (key.equalsIgnoreCase("NEUTRE_TOURELLE")) {
            int parsed = value.toInt();
            if (parsed == 0 && value != "0") {
                LOG_WARN("Invalid NEUTRE_TOURELLE value: %s (parsed as 0)", value.c_str());
            } else {
                robot.servoNeutralTurret = parsed;
            }
        } else if (key.equalsIgnoreCase("SERVO_DIR_MIN")) {
            int parsed = value.toInt();
            if (parsed == 0 && value != "0") {
                LOG_WARN("Invalid SERVO_DIR_MIN value: %s (parsed as 0)", value.c_str());
            } else {
                robot.servoDirMin = parsed;
            }
        } else if (key.equalsIgnoreCase("SERVO_DIR_MAX")) {
            int parsed = value.toInt();
            if (parsed == 0 && value != "0") {
                LOG_WARN("Invalid SERVO_DIR_MAX value: %s (parsed as 0)", value.c_str());
            } else {
                robot.servoDirMax = parsed;
            }
        } else if (key.equalsIgnoreCase("ANGLE_TETE_BASSE")) {
            int parsed = value.toInt();
            if (parsed == 0 && value != "0") {
                LOG_WARN("Invalid ANGLE_TETE_BASSE value: %s (parsed as 0)", value.c_str());
            } else {
                robot.servoAngleHeadDown = parsed;
            }
        } else if (key.equalsIgnoreCase("ANGLE_SOL")) {
            int parsed = value.toInt();
            if (parsed == 0 && value != "0") {
                LOG_WARN("Invalid ANGLE_SOL value: %s (parsed as 0)", value.c_str());
            } else {
                robot.servoAngleGround = parsed;
            }
        } else if (key.equalsIgnoreCase("ACCEL_RATE")) {
            float parsed = value.toFloat();
            if (parsed == 0.0 && !value.startsWith("0")) {
                LOG_WARN("Invalid ACCEL_RATE value: %s (parsed as 0.0)", value.c_str());
            } else {
                robot.accelRate = parsed;
            }
        } else if (key.equalsIgnoreCase("DIFF_STRENGTH")) {
            float parsed = value.toFloat();
            if (parsed == 0.0 && !value.startsWith("0")) {
                LOG_WARN("Invalid DIFF_STRENGTH value: %s (parsed as 0.0)", value.c_str());
            } else {
                robot.diffStrength = parsed;
            }
        } else if (key.equalsIgnoreCase("FWD_DIFF_COEFF")) {
            float parsed = value.toFloat();
            if (parsed == 0.0 && !value.startsWith("0")) {
                LOG_WARN("Invalid FWD_DIFF_COEFF value: %s (parsed as 0.0)", value.c_str());
            } else {
                robot.fwdDiffCoeff = parsed;
            }
        } else if (key.equalsIgnoreCase("AVOID_BACKUP_DURATION_MS")) {
            int parsed = value.toInt();
            if (parsed == 0 && value != "0") {
                LOG_WARN("Invalid AVOID_BACKUP_DURATION_MS value: %s (parsed as 0)", value.c_str());
            } else {
                robot.avoidBackupDuration = parsed;
            }
        } else if (key.equalsIgnoreCase("MIN_DIST_FOR_VALID_PATH")) {
            int parsed = value.toInt();
            if (parsed == 0 && value != "0") {
                LOG_WARN("Invalid MIN_DIST_FOR_VALID_PATH value: %s (parsed as 0)", value.c_str());
            } else {
                robot.minDistForValidPath = parsed;
            }
        } else if (key.equalsIgnoreCase("TURRET_MOVE_TIME_MS")) {
            int parsed = value.toInt();
            if (parsed == 0 && value != "0") {
                LOG_WARN("Invalid TURRET_MOVE_TIME_MS value: %s (parsed as 0)", value.c_str());
            } else {
                robot.turretMoveTime = parsed;
            }
        } else if (key.equalsIgnoreCase("SEUIL_VIDE")) {
            int parsed = value.toInt();
            if (parsed == 0 && value != "0") {
                LOG_WARN("Invalid SEUIL_VIDE value: %s (parsed as 0)", value.c_str());
            } else {
                robot.seuilVide = parsed;
            }
        } else if (key.equalsIgnoreCase("VL53L1X_TIMING_BUDGET_US")) {
            int parsed = value.toInt();
            if (parsed == 0 && value != "0") {
                LOG_WARN("Invalid VL53L1X_TIMING_BUDGET_US value: %s (parsed as 0)", value.c_str());
            } else {
                robot.laserTimingBudget = parsed;
            }
        } else if (key.equalsIgnoreCase("VL53L1X_INTER_MEASUREMENT_PERIOD_MS")) {
            int parsed = value.toInt();
            if (parsed == 0 && value != "0") {
                LOG_WARN("Invalid VL53L1X_INTER_MEASUREMENT_PERIOD_MS value: %s (parsed as 0)", value.c_str());
            } else {
                robot.laserInterMeasurementPeriod = parsed;
            }
        } else if (key.equalsIgnoreCase("INITIAL_AUTONOMOUS_DELAY_MS")) {
            int parsed = value.toInt();
            if (parsed == 0 && value != "0") {
                LOG_WARN("Invalid INITIAL_AUTONOMOUS_DELAY_MS value: %s (parsed as 0)", value.c_str());
            } else {
                robot.initialAutonomousDelay = parsed;
            }
        }
    }

    configFile.close();
    LOG_INFO("Config loaded.");
    return true;
}

SdFat* get_sd_card() {
    return &sd;
}