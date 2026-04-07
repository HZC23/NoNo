#include "hardware.h"
#include "robot.h"
#include "comms.h"
#include "logger.h"
#include <Preferences.h>
#include <cmath>


bool isNVSDataValid() {
    Preferences preferences;
    preferences.begin(NVS_NAMESPACE, true); // Use read-only mode
    int magicValue = preferences.getInt(NVS_COMPASS_MAGIC_KEY, 0);
    preferences.end();
    return (magicValue == NVS_MAGIC_VALUE);
}

void saveCompassCalibration(const Robot& robot) {
    LOG_INFO("Sauvegarde de la calibration compas en NVS...");
    Preferences preferences;
    preferences.begin(NVS_NAMESPACE, false); // Read and write

    preferences.putInt(NVS_COMPASS_MAGIC_KEY, NVS_MAGIC_VALUE);
    preferences.putFloat(NVS_COMPASS_OFFSET_KEY, robot.compassOffset);
    preferences.putBool(NVS_COMPASS_CALIBRATED_KEY, robot.compassCalibrated);
    preferences.putShort(NVS_MAG_MIN_X_KEY, robot.magMin.x);
    preferences.putShort(NVS_MAG_MIN_Y_KEY, robot.magMin.y);
    preferences.putShort(NVS_MAG_MIN_Z_KEY, robot.magMin.z);
    preferences.putShort(NVS_MAG_MAX_X_KEY, robot.magMax.x);
    preferences.putShort(NVS_MAG_MAX_Y_KEY, robot.magMax.y);
    preferences.putShort(NVS_MAG_MAX_Z_KEY, robot.magMax.z);

    preferences.end();
    LOG_INFO("Calibration compas sauvegardee en NVS.");
}

void loadCompassCalibration(Robot& robot) {
    if (isNVSDataValid()) {
        Preferences preferences;
        preferences.begin(NVS_NAMESPACE, true); // Read-only

        robot.compassOffset = preferences.getFloat(NVS_COMPASS_OFFSET_KEY, 0.0);
        robot.compassCalibrated = preferences.getBool(NVS_COMPASS_CALIBRATED_KEY, false);
        robot.magMin.x = preferences.getShort(NVS_MAG_MIN_X_KEY, COMPASS_MAX_INT16);
        robot.magMin.y = preferences.getShort(NVS_MAG_MIN_Y_KEY, COMPASS_MAX_INT16);
        robot.magMin.z = preferences.getShort(NVS_MAG_MIN_Z_KEY, COMPASS_MAX_INT16);
        robot.magMax.x = preferences.getShort(NVS_MAG_MAX_X_KEY, COMPASS_MIN_INT16);
        robot.magMax.y = preferences.getShort(NVS_MAG_MAX_Y_KEY, COMPASS_MIN_INT16);
        robot.magMax.z = preferences.getShort(NVS_MAG_MAX_Z_KEY, COMPASS_MIN_INT16);

        preferences.end();
        LOG_INFO("Calibration compas chargee depuis NVS.");
    } else {
        LOG_INFO("Aucune calibration valide trouvee en NVS.");
        robot.compassCalibrated = false;
    }
}

void compass_init(Robot& robot) {
    LOG_INFO("Initialisation du compas...");
    if (!compass->init()) {
        LOG_ERROR("Impossible d'initialiser le compas!");
        setLcdText(robot, "ERREUR COMPAS");
        robot.compassInitialized = false;
    } else {
        LOG_INFO("Compas initialise avec succes!");
        robot.compassInitialized = true;
        compass->enableDefault();
        loadCompassCalibration(robot);
    }
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

float getCalibratedHeading(Robot& robot) {
    if (!robot.compassInitialized) return std::nanf("");
    
    compass->read();
    if (compass->m.x == 0 && compass->m.y == 0 && compass->m.z == 0) {
        LOG_WARN("Valeurs compas nulles dans getCalibratedHeading ()");
        return robot.cap; // return last known heading
    }

    float heading;
    if (robot.compassCalibrated) {
        float corrected_x = compass->m.x - (robot.magMin.x + robot.magMax.x) / 2.0;
        float corrected_y = compass->m.y - (robot.magMin.y + robot.magMax.y) / 2.0;

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

    if (robot.compassInverted) {
        heading += 180.0;
    }
    heading += robot.compassOffset;

    while (heading < 0.0) heading += 360.0;
    while (heading >= 360.0) heading -= 360.0;
    
    return heading;
}

void calibrateCompass(Robot& robot) {
    static unsigned long startTime = 0;
    static bool calibrationStarted = false;

    if (!calibrationStarted) {
        LOG_INFO("=== CALIBRATION DU COMPAS (360 DEGRES) ===");
        robot.magMin = {COMPASS_MAX_INT16, COMPASS_MAX_INT16, COMPASS_MAX_INT16};
        robot.magMax = {COMPASS_MIN_INT16, COMPASS_MIN_INT16, COMPASS_MIN_INT16};
        startTime = millis();
        calibrationStarted = true;
        setLcdText(robot, "Calib 360...");
    }

    unsigned long elapsed = millis() - startTime;

    if (elapsed < COMPASS_CALIBRATION_DURATION_MS) {
        compass->read();
        robot.magMin.x = min(robot.magMin.x, compass->m.x);
        robot.magMin.y = min(robot.magMin.y, compass->m.y);
        robot.magMin.z = min(robot.magMin.z, compass->m.z);
        robot.magMax.x = max(robot.magMax.x, compass->m.x);
        robot.magMax.y = max(robot.magMax.y, compass->m.y);
        robot.magMax.z = max(robot.magMax.z, compass->m.z);
    } else {
        LOG_INFO("=== CALIBRATION TERMINEE ===");
        // Validate calibration by checking the range of magnetometer values on both axes
        int rangeX = robot.magMax.x - robot.magMin.x;
        int rangeY = robot.magMax.y - robot.magMin.y;
        
        if (rangeX >= COMPASS_CALIBRATION_VALIDATION_THRESHOLD && rangeY >= COMPASS_CALIBRATION_VALIDATION_THRESHOLD) {
            robot.compassCalibrated = true;
            LOG_INFO("-> Calibration REUSSIE. RangeX=%d, RangeY=%d", rangeX, rangeY);
            setLcdText(robot, "Calib. OK");
        } else {
            robot.compassCalibrated = false;
            LOG_WARN("-> Calibration DEFAILLANTE. RangeX=%d (need >=%d), RangeY=%d (need >=%d)", 
                     rangeX, COMPASS_CALIBRATION_VALIDATION_THRESHOLD, 
                     rangeY, COMPASS_CALIBRATION_VALIDATION_THRESHOLD);
            setLcdText(robot, "Calib. ECHEC");
        }
        saveCompassCalibration(robot);
        calibrationStarted = false;
        changeState(robot, IDLE); // Return to IDLE state
    } else if (elapsed > COMPASS_CALIBRATION_TIMEOUT_MS) {
        // Safety timeout to prevent getting stuck in this state forever
        LOG_ERROR("=== CALIBRATION TIMEOUT ===");
        setLcdText(robot, "Calib. TIMEOUT");
        calibrationStarted = false;
        changeState(robot, IDLE);
    }
}

float getPitch(Robot& robot) {
    if (!robot.compassInitialized) return 0.0f;
    compass->readAcc();
    return atan2(compass->a.y, -compass->a.z) * 180.0 / PI;
}

bool detectImpactOrStall(Robot& robot) {
    static float lastAccelX = 0;
    static float lastAccelY = 0;
    static int impactCount = 0;
    static bool accelInitialized = false;
    
    // Guard against uninitialized compass
    if (!robot.compassInitialized) {
        impactCount = 0;
        accelInitialized = false;
        return false;
    }
    
    compass->readAcc();
    float currentX = compass->a.x;
    float currentY = compass->a.y;
    
    // Initialize on first valid read to avoid startup false positives
    if (!accelInitialized) {
        lastAccelX = currentX;
        lastAccelY = currentY;
        accelInitialized = true;
        return false;
    }
    
    float jerk = abs(currentX - lastAccelX) + abs(currentY - lastAccelY);
    lastAccelX = currentX;
    lastAccelY = currentY;

    if (jerk > IMPACT_DETECTION_THRESHOLD) {
        impactCount++;
        if (impactCount >= IMPACT_THRESHOLD_COUNT) {
            LOG_WARN("IMU: IMPACT DETECTED!");
            impactCount = 0;
            return true;
        }
    } else {
        impactCount = 0;
    }
    return false;
}
