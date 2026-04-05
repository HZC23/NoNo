#include "hardware.h"
#include "robot.h"
#include "logger.h"

unsigned long lastTurretUpdate = 0;
int turretPanAngle = 90;
int turretTiltAngle = 90;
int turretScanDirection = 1;

void stabilizeTurretTilt(Robot& robot) {
    float pitch = getPitch(robot);
    // Apply a simple stability factor. The value '0.8' can be tuned.
    int targetTilt = 90 - (int)(pitch * 0.8);
    turretTiltAngle = constrain(targetTilt, 70, 110); // Physical limits of the servo
}

void updateTurret(Robot& robot, bool isMovingForward) {
    // If in manual mode, the joystick has full control. Do not run autonomous logic.
    if (robot.currentState == MANUAL_COMMAND_MODE) {
        return;
    }

    // Only update turret at a fixed interval to prevent jitter
    if (millis() - lastTurretUpdate > 30) {
        lastTurretUpdate = millis();

        // Stabilize tilt based on pitch to maintain balance
        stabilizeTurretTilt(robot);

        // Pan logic depends on the robot's state
        switch (robot.currentState) {
            case MOVING_FORWARD:
            case FOLLOW_HEADING:
            case MAINTAIN_HEADING:
            case SMART_AVOIDANCE: // Also scan while avoiding
                // Gentle side-to-side scanning when moving forward
                turretPanAngle += (2 * turretScanDirection);
                if (turretPanAngle >= 120 || turretPanAngle <= 60) {
                    turretScanDirection *= -1;
                }
                break;

            case OBSTACLE_AVOIDANCE:
                // Wider, faster scanning when an obstacle is detected and needs analysis
                turretPanAngle += (5 * turretScanDirection);
                if (turretPanAngle >= 170 || turretPanAngle <= 10) {
                    turretScanDirection *= -1;
                }
                break;
            
            case TURNING_LEFT:
            case TURNING_RIGHT:
                // This is handled by syncTurretWithSteering, no action needed here
                break;

            case IDLE:
            default:
                // Gently return to center when idle
                if (turretPanAngle > 90) turretPanAngle--;
                else if (turretPanAngle < 90) turretPanAngle++;
                // Note: turretTiltAngle is stabilized via stabilizeTurretTilt above (follows pitch)
                break;
        }
        
        // Use full 0-180 range for pan and tilt
        tourelle.write(constrain(turretPanAngle, 0, 180), constrain(turretTiltAngle, 0, 180));
    }
}

void syncTurretWithSteering(int steeringAngle) {
    // Make the turret "look" into the turn
    int headAngle = 90 + (steeringAngle - 90) * 1.5;
    turretPanAngle = constrain(headAngle, 10, 170);
    tourelle.write(turretPanAngle, turretTiltAngle);
}

int setAckermannAngle(Robot& robot, int angleError, int speed) {
    // This function is now used to control the steering servo for Ackermann steering.
    // The angleError and speed parameters are vestiges of a differential drive setup
    // and are not directly used here, as the target steering angle comes from the joystick.
    // Servodirection.write(robot.manualTargetSteeringAngle);
    // LOG_DEBUG("Ackermann Steering Angle: %d", robot.manualTargetSteeringAngle);
    return robot.manualTargetSteeringAngle; // Return the angle for potential external use or logging
}
