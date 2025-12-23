#include "sd_utils.h"
#include "config.h" // For SD_CS_PIN and DEBUG_MODE
#include "state.h"    // For Robot struct
#include <string.h>


// Helper to check if a string ends with a specific suffix
bool endsWith(const char* str, const char* suffix) {
    if (!str || !suffix) return 0;
    size_t lenstr = strlen(str);
    size_t lensuffix = strlen(suffix);
    if (lensuffix > lenstr) return 0;
    return strncmp(str + lenstr - lensuffix, suffix, lensuffix) == 0;
}

bool setupSDCard() {
    if (DEBUG_MODE) {
        Serial.print(F("Initializing SD card..."));
    }
    // Initialize SPI bus with specific pins
    SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS_PIN);
    if (!SD.begin(SD_CS_PIN)) {
        if (DEBUG_MODE) {
            Serial.println(F("SD Card initialization failed!"));
        }
        return false;
    }
    if (DEBUG_MODE) {
        Serial.println(F("SD Card initialized."));
    }
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
        line.trim(); // Remove leading/trailing whitespace and newline

        if (line.isEmpty() || line.startsWith("#")) {
            continue; // Skip empty lines and comments
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
            if (DEBUG_MODE) Serial.printf("  VITESSE_MOYENNE = %d\n", robot.speedAvg);
        } else if (key.equalsIgnoreCase("VITESSE_LENTE")) {
            robot.speedSlow = value.toInt();
            if (DEBUG_MODE) Serial.printf("  VITESSE_LENTE = %d\n", robot.speedSlow);
        } else if (key.equalsIgnoreCase("VITESSE_ROTATION")) {
            robot.speedRotation = value.toInt();
            if (DEBUG_MODE) Serial.printf("  VITESSE_ROTATION = %d\n", robot.speedRotation);
        } else if (key.equalsIgnoreCase("TOLERANCE_VIRAGE")) {
            robot.turnTolerance = value.toFloat();
            if (DEBUG_MODE) Serial.printf("  TOLERANCE_VIRAGE = %.2f\n", robot.turnTolerance);
        } else if (key.equalsIgnoreCase("KP_HEADING")) {
            robot.KpHeading = value.toFloat();
            if (DEBUG_MODE) Serial.printf("  KP_HEADING = %.2f\n", robot.KpHeading);
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
        } else {
            if (DEBUG_MODE) {
                Serial.print(F("Unknown config key: "));
                Serial.println(key);
            }
        }
    }

    configFile.close();
    if (DEBUG_MODE) {
        Serial.println(F("Config loaded."));
    }
    return true;
}