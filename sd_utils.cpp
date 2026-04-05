#include "hardware.h"
#include "robot.h"
#include "logger.h"
#include <cstring>  // For strchr, strncpy, strcasecmp, strtok_r
#include <cctype>   // For isspace

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

// Helper function to trim leading/trailing whitespace (in place)
static void trim_string(char* str) {
    if (str == nullptr) return;
    
    // Trim leading whitespace
    char* start = str;
    while (*start && isspace((unsigned char)*start)) start++;
    
    // Handle empty string after trimming leading whitespace
    if (*start == '\0') {
        str[0] = '\0';
        return;
    }
    
    // Trim trailing whitespace
    char* end = str + strlen(start) - 1;
    while (end >= start && isspace((unsigned char)*end)) end--;
    
    // Shift and null terminate
    size_t len = end - start + 1;
    memmove(str, start, len);
    str[len] = '\0';
}

// Helper to check if a line is empty or a comment
static bool should_skip_line(const char* line) {
    while (*line && isspace((unsigned char)*line)) line++;
    return *line == '\0' || *line == '#';
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

    // Single line buffer - reused for each config line
    char lineBuffer[128];
    size_t lineLen = 0;

    while (configFile.available()) {
        // Read one line into buffer
        lineLen = 0;
        while (configFile.available() && lineLen < sizeof(lineBuffer) - 1) {
            char c = configFile.read();
            if (c == '\n' || c == '\r') {
                if (c == '\r' && configFile.peek() == '\n') {
                    configFile.read(); // consume '\n'
                }
                break;
            }
            lineBuffer[lineLen++] = c;
        }
        lineBuffer[lineLen] = '\0';

        // Trim the line in place
        trim_string(lineBuffer);

        // Skip empty lines and comments
        if (should_skip_line(lineBuffer)) {
            continue;
        }

        // Parse "KEY=VALUE" format using standard C string functions
        char* equalsPos = strchr(lineBuffer, '=');
        if (equalsPos == nullptr) {
            LOG_WARN("Invalid config line (no '='): %s", lineBuffer);
            continue;
        }

        // Split key and value - prevent buffer overflow
        size_t keyLen = equalsPos - lineBuffer;
        char key[64];
        size_t copyLen = (keyLen < sizeof(key) - 1) ? keyLen : (sizeof(key) - 1);
        strncpy(key, lineBuffer, copyLen);
        key[copyLen] = '\0';  // Ensure null termination
        trim_string(key);

        char* valuePtr = equalsPos + 1;
        // Create a value buffer for trimming
        char value[64];
        strncpy(value, valuePtr, sizeof(value) - 1);
        value[sizeof(value) - 1] = '\0';
        trim_string(value);

        // Skip empty values
        if (value[0] == '\0') {
            LOG_WARN("Empty config value for key: %s", key);
            continue;
        }

        // Process each config key using standard C case-insensitive compare
#define CONFIG_INT(KEY_NAME, ROBOT_FIELD) \
        if (strcasecmp(key, KEY_NAME) == 0) { \
            int parsed = atoi(value); \
            if (parsed == 0 && strcmp(value, "0") != 0) { \
                LOG_WARN("Invalid " KEY_NAME " value: %s (parsed as 0)", value); \
            } else { \
                robot.ROBOT_FIELD = parsed; \
            } \
            continue; \
        }

#define CONFIG_FLOAT(KEY_NAME, ROBOT_FIELD) \
        if (strcasecmp(key, KEY_NAME) == 0) { \
            char* endptr; \
            float parsed = strtof(value, &endptr); \
            if (endptr == value || (*endptr != '\0' && !isspace((unsigned char)*endptr))) { \
                LOG_WARN("Invalid " KEY_NAME " value: %s (parse failed)", value); \
            } else { \
                robot.ROBOT_FIELD = parsed; \
            } \
            continue; \
        }

        // Integer configs
        CONFIG_INT("VITESSE_MOYENNE", speedAvg);
        CONFIG_INT("VITESSE_LENTE", speedSlow);
        CONFIG_INT("VITESSE_ROTATION", speedRotation);
        CONFIG_INT("VITESSE_ROTATION_MAX", speedRotationMax);
        CONFIG_INT("MIN_SPEED_TO_MOVE", minSpeedToMove);
        CONFIG_INT("NEUTRE_DIRECTION", servoNeutralDir);
        CONFIG_INT("NEUTRE_TOURELLE", servoNeutralTurret);
        CONFIG_INT("SERVO_DIR_MIN", servoDirMin);
        CONFIG_INT("SERVO_DIR_MAX", servoDirMax);
        CONFIG_INT("ANGLE_TETE_BASSE", servoAngleHeadDown);
        CONFIG_INT("ANGLE_SOL", servoAngleGround);
        CONFIG_INT("AVOID_BACKUP_DURATION_MS", avoidBackupDuration);
        CONFIG_INT("MIN_DIST_FOR_VALID_PATH", minDistForValidPath);
        CONFIG_INT("TURRET_MOVE_TIME_MS", turretMoveTime);
        CONFIG_INT("SEUIL_VIDE", seuilVide);
        CONFIG_INT("VL53L1X_TIMING_BUDGET_US", laserTimingBudget);
        CONFIG_INT("VL53L1X_INTER_MEASUREMENT_PERIOD_MS", laserInterMeasurementPeriod);
        CONFIG_INT("INITIAL_AUTONOMOUS_DELAY_MS", initialAutonomousDelay);

        // Float configs
        CONFIG_FLOAT("TOLERANCE_VIRAGE", turnTolerance);
        CONFIG_FLOAT("KP_HEADING", KpHeading);
        CONFIG_FLOAT("CALIBRATION_MOTEUR_B", motorBCalibration);
        CONFIG_FLOAT("SEUIL_BASCULE_DIRECTION", pivotAngleThreshold);
        CONFIG_FLOAT("ACCEL_RATE", accelRate);
        CONFIG_FLOAT("DIFF_STRENGTH", diffStrength);
        CONFIG_FLOAT("FWD_DIFF_COEFF", fwdDiffCoeff);

        #undef CONFIG_INT
        #undef CONFIG_FLOAT

        // Log if unknown key
        LOG_WARN("Unknown config key: %s", key);
    }

    configFile.close();
    LOG_INFO("Config loaded.");
    return true;
}

SdFat* get_sd_card() {
    return &sd;
}