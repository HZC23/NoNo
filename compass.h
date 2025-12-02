#ifndef COMPASS_H
#define COMPASS_H

#include <Arduino.h>
#include <LSM303.h>
#include <EEPROM.h>
#include "state.h"
#include "display.h" // For setLcdText

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

// Function Prototypes
void compass_init(Robot& robot);
float getCalibratedHeading(Robot& robot);
void calibrateCompass(Robot& robot);
void saveCompassCalibration(const Robot& robot);
void loadCompassCalibration(Robot& robot);
bool isEEPROMDataValid();
float calculateHeading(float y, float x); // Overloaded
float calculateHeading(const LSM303& compass);
float getPitchAngle(Robot& robot); // New prototype for pitch calculation
void displayCompassInfo(Robot& robot);


// --- IMPLEMENTATIONS ---

// New function to calculate pitch angle from accelerometer
inline float getPitchAngle(Robot& robot) {
    // Read accelerometer data
    compass.readAcc();

    // Calculate pitch using atan2 for robustness
    // The accelerometer measures gravity, so an un-tilted sensor will read (0,0,1g) on Z if Z is up.
    // For pitch (rotation around X-axis), we use Y and Z components.
    // Assuming Z is vertical and Y is forward/backward.
    // Adjust signs based on sensor orientation.
    // Angle = atan2(y, z) * 180 / PI; (standard formula)
    // Adjust 90-degree offset if sensor is mounted horizontally
    float pitch = atan2(compass.a.y, compass.a.z) * 180.0 / PI;

    // Compensate for sensor mounting (e.g., if Z points forward and Y up)
    // This part often requires calibration or knowing the exact sensor orientation.
    // For simplicity, let's assume it's mounted such that pitch is directly related to atan2(y, z)

    return pitch;
}

// Overloaded function to calculate heading from y and x components
inline float calculateHeading(float y, float x) {
  float heading = (atan2(y, x) * 180) / PI;
  if (heading < 0) {
    heading += 360;
  }
  return heading;;
}

inline float calculateHeading(const LSM303& compass) {
  return calculateHeading(compass.m.y, compass.m.x);
}

inline void compass_init(Robot& robot) {
    if (DEBUG_MODE) Serial.println("Initialisation du compas...");
    if (!compass.init()) {
        if (DEBUG_MODE) Serial.println("ERREUR: Impossible d'initialiser le compas!");
        setLcdText(robot, "ERREUR COMPAS");
        robot.compassInitialized = false;
    } else {
        if (DEBUG_MODE) Serial.println("Compas initialise avec succes!");
        robot.compassInitialized = true;
        compass.enableDefault();
        loadCompassCalibration(robot);
    }
}

inline float getCalibratedHeading(Robot& robot) {
    if (!robot.compassInitialized) return 0.0;
    
    compass.read();
    if (compass.m.x == 0 && compass.m.y == 0 && compass.m.z == 0) {
        if (DEBUG_MODE) Serial.println("ATTENTION: Valeurs compas nulles dans getCalibratedHeading()");
        return 0.0;
    }

    float heading;
    if (robot.compassCalibrated) {
        // Correct for hard-iron distortion
        float corrected_x = compass.m.x - (robot.magMin.x + robot.magMax.x) / 2.0;
        float corrected_y = compass.m.y - (robot.magMin.y + robot.magMax.y) / 2.0;

        // Correct for soft-iron distortion
        float avg_delta_x = (robot.magMax.x - robot.magMin.x) / 2.0;
        float avg_delta_y = (robot.magMax.y - robot.magMin.y) / 2.0;
        
        if (avg_delta_x == 0 || avg_delta_y == 0) {
            heading = calculateHeading(compass);
        } else {
            float avg_delta = (avg_delta_x + avg_delta_y) / 2.0;
            float scaled_x = corrected_x * (avg_delta / avg_delta_x);
            float scaled_y = corrected_y * (avg_delta / avg_delta_y);
            heading = calculateHeading(scaled_y, scaled_x);
        }
    } else {
        heading = calculateHeading(compass);
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

inline void calibrateCompass(Robot& robot) {
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
        compass.read();
        robot.magMin.x = min(robot.magMin.x, compass.m.x);
        robot.magMin.y = min(robot.magMin.y, compass.m.y);
        robot.magMin.z = min(robot.magMin.z, compass.m.z);
        robot.magMax.x = max(robot.magMax.x, compass.m.x);
        robot.magMax.y = max(robot.magMax.y, compass.m.y);
        robot.magMax.z = max(robot.magMax.z, compass.m.z);
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

inline void saveCompassCalibration(const Robot& robot) {
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
}

inline void loadCompassCalibration(Robot& robot) {
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

inline bool isEEPROMDataValid() {
    int magicValue = 0;
    EEPROM.get(EEPROM_MAGIC_NUMBER_ADDR, magicValue);
    return (magicValue == EEPROM_MAGIC_VALUE);
}

inline void displayCompassInfo(Robot& robot) {
    float heading = getCalibratedHeading(robot);
    const char* direction;

    if (heading >= CARDINAL_NORTH_LOWER || heading < CARDINAL_NORTH_UPPER) {
        direction = "N";
    } else if (heading < CARDINAL_NORTHEAST_UPPER) {
        direction = "NE";
    } else if (heading < CARDINAL_EAST_UPPER) {
        direction = "E";
    } else if (heading < CARDINAL_SOUTHEAST_UPPER) {
        direction = "SE";
    } else if (heading < CARDINAL_SOUTH_UPPER) {
        direction = "S";
    } else if (heading < CARDINAL_SOUTHWEST_UPPER) {
        direction = "SO";
    } else if (heading < CARDINAL_WEST_UPPER) {
        direction = "O";
    } else {
        direction = "NO";
    }

    char textBuffer[MAX_LCD_TEXT_LENGTH + 1];
    snprintf(textBuffer, sizeof(textBuffer), "%s %ddeg", direction, (int)heading);
    setLcdText(robot, textBuffer);
}

#endif // COMPASS_H