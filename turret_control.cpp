#include "turret_control.h"

// --- Turret State Variables ---
unsigned long lastTurretUpdate = 0;
int turretPanAngle = 90;
int turretTiltAngle = 90;
int turretScanDirection = 1;

/**
 * @brief Stabilizes the turret's vertical tilt based on the robot's pitch.
 */
void stabilizeTurretTilt(Robot& robot) {
    float pitch = getPitch(robot);
    int targetTilt = 90 - (int)(pitch * TURRET_TILT_STABILITY_FACTOR);
    turretTiltAngle = constrain(targetTilt, 70, 110);
}

/**
 * @brief Main non-blocking function to update the turret's position and scan behavior.
 */
void updateTurret(Robot& robot, bool isMovingForward) {
    if (millis() - lastTurretUpdate > TURRET_UPDATE_INTERVAL) {
        lastTurretUpdate = millis();

        stabilizeTurretTilt(robot);

        switch (robot.currentState) {
            case MOVING_FORWARD:
            case FOLLOW_HEADING:
            case MAINTAIN_HEADING:
                turretPanAngle += (PREDICTIVE_SCAN_STEP * turretScanDirection);
                if (turretPanAngle >= PREDICTIVE_SCAN_MAX_ANGLE || turretPanAngle <= PREDICTIVE_SCAN_MIN_ANGLE) {
                    turretScanDirection *= -1;
                }
                break;

            case OBSTACLE_AVOIDANCE:
                turretPanAngle += (WIDE_SCAN_STEP * turretScanDirection);
                if (turretPanAngle >= WIDE_SCAN_MAX_ANGLE || turretPanAngle <= WIDE_SCAN_MIN_ANGLE) {
                    turretScanDirection *= -1;
                }
                break;
            
            case TURNING_LEFT:
            case TURNING_RIGHT:
                // "Look-Ahead" is handled by handleFollowHeadingState, which calls syncTurretWithSteering.
                // No automatic scanning during a commanded turn.
                break;

            case IDLE:
            default:
                // Gently move towards a neutral/resting position.
                if(turretPanAngle > 90) turretPanAngle--;
                else if (turretPanAngle < 90) turretPanAngle++;
                if(turretTiltAngle > 95) turretTiltAngle--;
                else if (turretTiltAngle < 95) turretTiltAngle++;
                break;
        }
        
        tourelle.write(constrain(turretPanAngle, WIDE_SCAN_MIN_ANGLE, WIDE_SCAN_MAX_ANGLE), turretTiltAngle);
    }
}

/**
 * @brief Points the turret in the direction of a turn (Ackermann steering).
 */
void syncTurretWithSteering(int steeringAngle) {
    int headAngle = 90 + (steeringAngle - 90) * 1.5;
    turretPanAngle = constrain(headAngle, WIDE_SCAN_MIN_ANGLE, WIDE_SCAN_MAX_ANGLE);
    tourelle.write(turretPanAngle, turretTiltAngle); // Preserve current tilt
}

/**
 * @brief Performs a 180-degree scan to find the direction with the most free space.
 */
int findClearestPath() {
    int bestAngle = -1;
    int maxDistance = 0;
    
    if (DEBUG_MODE) Serial.println("Scanning for clearest path...");

    for (int angle = WIDE_SCAN_MIN_ANGLE; angle <= WIDE_SCAN_MAX_ANGLE; angle += 10) {
        tourelle.write(angle, turretTiltAngle); // Preserve current tilt
        delay(50);
        int currentDistance = vl53->readRangeContinuousMillimeters() / 10;
        if (currentDistance == 0) currentDistance = 500; // Treat out-of-range as far
        
        if (DEBUG_MODE) {
            Serial.printf("  Angle: %d, Distance: %d cm\n", angle, currentDistance);
        }

        if (currentDistance > maxDistance) {
            maxDistance = currentDistance;
            bestAngle = angle;
        }
    }
    
    if (bestAngle != -1) {
        tourelle.write(bestAngle, turretTiltAngle);
    } else {
        tourelle.write(90, turretTiltAngle);
    }
    
    if (DEBUG_MODE) {
        Serial.printf("Scan complete. Best Angle: %d, Max Distance: %d cm. Threshold: %d cm\n", 
                      bestAngle, maxDistance, LASER_OBSTACLE_THRESHOLD_CM);
        Serial.printf("Returning: %d\n", (maxDistance > LASER_OBSTACLE_THRESHOLD_CM) ? bestAngle : -1);
    }

    // Only return a valid angle if the path is reasonably clear
    return (maxDistance > LASER_OBSTACLE_THRESHOLD_CM) ? bestAngle : -1;
}