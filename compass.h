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
float calculateHeading(const LSM303& compass);
void displayCompassInfo(Robot& robot);


// --- IMPLEMENTATIONS ---

inline float calculateHeading(const LSM303& compass) {
  float heading = (atan2(compass.m.y, compass.m.x) * 180) / PI;
  if (heading < 0) {
    heading += 360;
  }
  return heading;
}

inline void compass_init(Robot& robot) {
    if (DEBUG_MODE) Serial.println("Initialisation du compas...");
    if (!compass.init()) {
        if (DEBUG_MODE) Serial.println("ERREUR: Impossible d'initialiser le compas!");
        setLcdText("ERREUR COMPAS");
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

    if (robot.compassCalibrated) {
        float corrected_x = compass.m.x - (robot.magMin.x + robot.magMax.x) / 2.0;
        float corrected_y = compass.m.y - (robot.magMin.y + robot.magMax.y) / 2.0;

        float avg_delta_x = (robot.magMax.x - robot.magMin.x) / 2.0;
        float avg_delta_y = (robot.magMax.y - robot.magMin.y) / 2.0;
        
        if (avg_delta_x == 0 || avg_delta_y == 0) {
            return calculateHeading(compass);
        }

        float avg_delta = (avg_delta_x + avg_delta_y) / 2.0;
        float scaled_x = corrected_x * (avg_delta / avg_delta_x);
        float scaled_y = corrected_y * (avg_delta / avg_delta_y);

        float heading = (atan2(scaled_y, scaled_x) * 180) / PI;
        if (heading < 0) {
            heading += 360;
        }
        
        if (robot.compassInverted) {
            heading += 180.0;
            if (heading >= 360.0) {
                heading -= 360.0;
            }
        }
        return heading;
    } else {
        return calculateHeading(compass);
    }
}

inline void calibrateCompass(Robot& robot) {
    static unsigned long startTime = 0;
    const unsigned long CALIBRATION_DURATION_MS = 15000;
    static bool calibrationStarted = false;

    if (!calibrationStarted) {
        Serial.println("=== CALIBRATION DU COMPAS (360 DEGRES) ===");
        robot.magMin = {32767, 32767, 32767};
        robot.magMax = {-32768, -32768, -32768};
        startTime = millis();
        calibrationStarted = true;
        setLcdText("Calib 360...");
    }

    if (millis() - startTime < CALIBRATION_DURATION_MS) {
        compass.read();
        robot.magMin.x = min(robot.magMin.x, compass.m.x);
        robot.magMin.y = min(robot.magMin.y, compass.m.y);
        robot.magMin.z = min(robot.magMin.z, compass.m.z);
        robot.magMax.x = max(robot.magMax.x, compass.m.x);
        robot.magMax.y = max(robot.magMax.y, compass.m.y);
        robot.magMax.z = max(robot.magMax.z, compass.m.z);
    } else {
        Serial.println("=== CALIBRATION TERMINEE ===");
        if (robot.magMin.x < 32000 && robot.magMax.x > -32000) {
            robot.compassCalibrated = true;
            Serial.println("✅ Calibration REUSSIE.");
            setLcdText("Calib. OK");
        } else {
            robot.compassCalibrated = false;
            Serial.println("⚠️ Calibration DEFAILLANTE.");
            setLcdText("Calib. ECHEC");
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
    String direction;
    if (heading >= 337.5 || heading < 22.5) direction = "N";
    else if (heading >= 22.5 && heading < 67.5) direction = "NE";
    else if (heading >= 67.5 && heading < 112.5) direction = "E";
    else if (heading >= 112.5 && heading < 157.5) direction = "SE";
    else if (heading >= 157.5 && heading < 202.5) direction = "S";
    else if (heading >= 202.5 && heading < 247.5) direction = "SO";
    else if (heading >= 247.5 && heading < 292.5) direction = "O";
    else direction = "NO";
    
    String text = direction + " " + String((int)heading) + "deg";
    setLcdText(text);
}

#endif // COMPASS_H