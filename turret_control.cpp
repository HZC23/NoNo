#include "hardware.h"
#include "robot.h"
#include "logger.h"

// --- Helper for Turret Pitch Stabilization ---
void stabilizeTurretTilt(Robot& robot) {
    float pitch = getPitch(robot);
    // Apply a simple stability factor. The value '0.8' can be tuned.
    int targetTilt = 90 - (int)(pitch * 0.8);
    robot.turretTiltAngle = constrain(targetTilt, 70, 110); // Physical limits of the servo
}

// --- Main Turret Update Logic ---
void updateTurret(Robot& robot) {
    // 1. Guard: Check if servos are attached
    if (!tourelle.isAttached()) {
        return;
    }

    // 2. Guard: Check if an active scan is currently using the turret
    // We assume that states like SMART_AVOIDANCE (during its sub-states) and OBSTACLE_AVOIDANCE
    // might be performing their own direct tourelle.write() calls.
    // However, to be consistent, we should centralize even those OR make updateTurret respect them.
    
    // For SMART_AVOIDANCE, it uses sub-states for scanning.
    if (robot.currentState == SMART_AVOIDANCE || robot.currentState == OBSTACLE_AVOIDANCE) {
        // We only update tilt stabilization, but let the state machine handle PAN
        if (millis() - robot.lastTurretUpdateTime > 50) {
            stabilizeTurretTilt(robot);
            // We DON'T call tourelle.write here to avoid fighting the state machine's scan
            // But if the state machine is NOT currently moving it (e.g. AVOID_INIT), we might want a default.
            // For now, let's assume the state machine is boss during avoidance.
            if (robot.obstacleAvoidanceState == AVOID_MOVE_FORWARD || robot.obstacleAvoidanceState == AVOID_IDLE) {
                // Not actively scanning, can center or look forward
                 robot.turretPanAngle = 90;
                 tourelle.write(robot.turretPanAngle, robot.turretTiltAngle);
            }
            robot.lastTurretUpdateTime = millis();
        }
        return;
    }

    // 3. Normal updates at a fixed interval
    if (millis() - robot.lastTurretUpdateTime > 30) {
        robot.lastTurretUpdateTime = millis();

        // Stabilize tilt based on pitch
        stabilizeTurretTilt(robot);

        // Pan logic depends on the robot's state
        switch (robot.currentState) {
            case MOVING_FORWARD:
            case FOLLOW_HEADING:
            case MAINTAIN_HEADING:
                // Gentle side-to-side scanning when moving forward
                robot.turretPanAngle += (2 * robot.turretScanDirection);
                if (robot.turretPanAngle >= 120 || robot.turretPanAngle <= 60) {
                    robot.turretScanDirection *= -1;
                }
                break;
            
            case MANUAL_COMMAND_MODE:
                if (robot.manualTargetVelocity != 0) {
                    // Look into the turn
                    if (abs(robot.manualTargetSteeringAngle - robot.servoNeutralDir) > 5) {
                        // Map steering angle to turret angle
                        int angleOffset = (robot.manualTargetSteeringAngle - robot.servoNeutralDir);
                        // If steering left (angle < neutral), look left (turret > 90)
                        // Note: Check if steering angle and turret angle directions match
                        int targetPan = 90 - (angleOffset * 1.2); 
                        robot.turretPanAngle = constrain(targetPan, 30, 150);
                    } else {
                        // Forward/Backward movement - gentle scan
                        robot.turretPanAngle += (1 * robot.turretScanDirection);
                        if (robot.turretPanAngle >= 105 || robot.turretPanAngle <= 75) {
                            robot.turretScanDirection *= -1;
                        }
                    }
                } else if (robot.manualTargetTurn != 0) {
                    // Pivoting in place - look in the direction of rotation
                    if (robot.manualTargetTurn > 0) robot.turretPanAngle = 130; // Looking left
                    else robot.turretPanAngle = 50; // Looking right
                } else {
                    // Stopped - return to center
                    if (robot.turretPanAngle > 92) robot.turretPanAngle -= 2;
                    else if (robot.turretPanAngle < 88) robot.turretPanAngle += 2;
                    else robot.turretPanAngle = 90;
                }
                break;

            case TURNING_LEFT:
                robot.turretPanAngle = 135;
                break;
            case TURNING_RIGHT:
                robot.turretPanAngle = 45;
                break;

            case CALIBRATING_COMPASS:
                // Look forward or slightly down during calibration
                robot.turretPanAngle = 90;
                robot.turretTiltAngle = 100;
                break;

            case IDLE:
            default:
                // Gently return to center
                if (robot.turretPanAngle > 92) robot.turretPanAngle -= 1;
                else if (robot.turretPanAngle < 88) robot.turretPanAngle += 1;
                else robot.turretPanAngle = 90;
                break;
        }
        
        tourelle.write(robot.turretPanAngle, robot.turretTiltAngle);
    }
}

void syncTurretWithSteering(int steeringAngle) {
    // Deprecated in favor of internal updateTurret logic for consistency,
    // but kept for compatibility if needed.
    int headAngle = 90 + (steeringAngle - 90) * 1.2;
    tourelle.write(constrain(headAngle, 30, 150), 90);
}
