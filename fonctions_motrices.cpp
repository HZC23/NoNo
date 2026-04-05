#include "fonctions_motrices.h"
#include "robot.h"
#include "logger.h"
#include "comms.h"
#include <cmath>

// Forward declaration
void calculateManualPwm(Robot& robot, int& outPwmA, int& outPwmB);

// --- Function Implementations ---
const char* stateToString(RobotState state) {
  switch (state) {
    case IDLE: return "IDLE";
    case MOVING_FORWARD: return "MOVING_FORWARD";
    case MOVING_BACKWARD: return "MOVING_BACKWARD";
    case TURNING_LEFT: return "TURNING_LEFT";
    case TURNING_RIGHT: return "TURNING_RIGHT";
    case MANUAL_FORWARD: return "MANUAL_FORWARD";
    case MANUAL_BACKWARD: return "MANUAL_BACKWARD";
    case MANUAL_TURNING_LEFT: return "MANUAL_TURNING_LEFT";
    case MANUAL_TURNING_RIGHT: return "MANUAL_TURNING_RIGHT";
    case OBSTACLE_AVOIDANCE: return "OBSTACLE_AVOIDANCE";
    case WAITING_FOR_TURRET: return "WAITING_FOR_TURRET";
    case FOLLOW_HEADING: return "FOLLOW_HEADING";
    case MAINTAIN_HEADING: return "MAINTAIN_HEADING";
    case BACKING_UP_OBSTACLE: return "BACKING_UP_OBSTACLE";
    case SCANNING_FOR_PATH: return "SCANNING_FOR_PATH";
    case TURNING_TO_PATH: return "TURNING_TO_PATH";
    case SMART_TURNING: return "SMART_TURNING";
    case CALIBRATING_COMPASS: return "CALIBRATING_COMPASS";
    case SMART_AVOIDANCE: return "SMART_AVOIDANCE";
    case SENTRY_MODE: return "SENTRY_MODE";
    case CHECKING_GROUND: return "CHECKING_GROUND";
    case CLIFF_DETECTED: return "CLIFF_DETECTED";
    case ANIMATING_HEAD: return "ANIMATING_HEAD";
    case GAMEPAD_CONTROL: return "GAMEPAD_CONTROL";
    case MANUAL_COMMAND_MODE: return "MANUAL_COMMAND_MODE";
    case EMERGENCY_EVASION: return "EMERGENCY_EVASION";
    case STUCK: return "STUCK";
    default: return "UNKNOWN_STATE";
  }
}

void changeState(Robot& robot, RobotState newState, ObstacleAvoidanceState avoidState) {
  if (robot.currentState == newState && newState != OBSTACLE_AVOIDANCE) return;
  
  trigger_rumble(75, 0, 255); // MAX haptic feedback for state change

  // --- Servo Power Management ---
  if (newState == IDLE) {
    LOG_DEBUG("Detaching servos for IDLE state.");
    tourelle.detach();
  } else if (robot.currentState == IDLE) {
    LOG_DEBUG("Re-attaching servos and setting to neutral.");
    tourelle.attach();
    tourelle.write(robot.servoNeutralTurret, robot.servoNeutralTurret);
  }
  
  robot.currentState = newState;
  robot.actionStarted = false;
  robot.lastActionTime = millis();
  
  if (newState == IDLE) {
    Arret();
    robot.currentPwmA = 0;
    robot.currentPwmB = 0;
    robot.curiousTargetH = robot.servoNeutralTurret;
    robot.curiousTargetV = robot.servoNeutralTurret;
  }

  if (newState == OBSTACLE_AVOIDANCE || newState == SMART_AVOIDANCE) {
    robot.consecutiveAvoidManeuvers++;
    robot.obstacleAvoidanceState = AVOID_INIT;
  } else {
    robot.consecutiveAvoidManeuvers = 0;
    robot.obstacleAvoidanceState = avoidState;
  }

  LOG_INFO("State -> %s", stateToString(newState));
}

int applySpeedRamp(Robot& robot, float target, float current) {
    if (abs(target - current) < 1.0) return (int)target;
    return (int)(current + (target - current) * robot.accelRate);
}

bool isObstacleDetected(Robot& robot) {
    bool ultraSonicObstacle = (robot.dusm < ULTRASONIC_OBSTACLE_THRESHOLD_CM && robot.dusm > 0);
    bool laserObstacle = (robot.laserInitialized && robot.distanceLaser < LASER_OBSTACLE_THRESHOLD_CM && robot.distanceLaser > 0);
    robot.obstacleDetectedByLaser = laserObstacle;
    return ultraSonicObstacle || laserObstacle;
}

float calculateHeadingError(float target, float current) {
  float error = target - current;
  if (error > 180) error -= 360;
  else if (error < -180) error += 360;
  return error;
}

void handleMovingForwardState(Robot& robot, int& targetA, int& targetB) {
  if (isObstacleDetected(robot)) {
      changeState(robot, SMART_AVOIDANCE);
      return;
  }
  // Simplified: other checks like stall/cliff are omitted for clarity
  targetA = robot.targetSpeed;
  targetB = robot.targetSpeed;
}

void handleFollowHeadingState(Robot& robot, int& targetA, int& targetB) {
  if (isObstacleDetected(robot)) {
      changeState(robot, SMART_AVOIDANCE);
      return;
  }
  // Simplified
  float currentHeading = getCalibratedHeading(robot);
  if (std::isnan(currentHeading)) {
      // Compass not initialized, use last known heading
      currentHeading = robot.cap;
  }
  float error = calculateHeadingError(robot.capCibleRotation, currentHeading);
  if (abs(error) < robot.turnTolerance) {
      changeState(robot, MOVING_FORWARD);
      return;
  }
  int pivotSpeed = robot.targetSpeed;
  if (error > 0) { targetA = -pivotSpeed; targetB = pivotSpeed; }
  else { targetA = pivotSpeed; targetB = -pivotSpeed; }
}

void handleSmartAvoidanceState(Robot& robot, int& targetA, int& targetB) {
  targetA = 0; // Default to stopping
  targetB = 0;
  const int QUICK_SCAN_ANGLE = 45; // 45 degrees left/right of center
  const uint32_t LASER_DATA_TIMEOUT_MS = 500; // Timeout for waiting for laser data

  switch (robot.obstacleAvoidanceState) {
    case AVOID_INIT:
      LOG_DEBUG("AVOID: INIT - Stopping and preparing to scan.");
      Arret();
      robot.actionStarted = false;
      robot.obstacleAvoidanceState = AVOID_QUICK_SCAN_LEFT;
      break;

    case AVOID_QUICK_SCAN_LEFT:
      if (!robot.actionStarted) {
        tourelle.write(90 - QUICK_SCAN_ANGLE, robot.servoNeutralTurret);
        robot.lastActionTime = millis();
        robot.actionStarted = true;
      }
      // Check if turret has moved and data is ready, or if we've timed out waiting
      if (millis() - robot.lastActionTime > robot.turretMoveTime) {
        if (robot.laserInitialized && vl53->dataReady()) {
          robot.quickScanLeftDist = vl53->readRangeContinuousMillimeters() / 10;
          LOG_DEBUG("AVOID: Quick scan LEFT distance: %d cm", robot.quickScanLeftDist);
          robot.actionStarted = false;
          robot.obstacleAvoidanceState = AVOID_QUICK_SCAN_RIGHT;
        } else if (millis() - robot.lastActionTime > robot.turretMoveTime + LASER_DATA_TIMEOUT_MS) {
          // Timeout waiting for laser data
          LOG_WARN("AVOID: Timeout waiting for LEFT laser data");
          robot.quickScanLeftDist = robot.maxUltrasonicDistance; // Safe fallback
          robot.actionStarted = false;
          robot.obstacleAvoidanceState = AVOID_QUICK_SCAN_RIGHT;
        }
      }
      break;

    case AVOID_QUICK_SCAN_RIGHT:
      if (!robot.actionStarted) {
        tourelle.write(90 + QUICK_SCAN_ANGLE, robot.servoNeutralTurret);
        robot.lastActionTime = millis();
        robot.actionStarted = true;
      }
      // Check if turret has moved and data is ready, or if we've timed out waiting
      if (millis() - robot.lastActionTime > robot.turretMoveTime) {
        if (robot.laserInitialized && vl53->dataReady()) {
          robot.quickScanRightDist = vl53->readRangeContinuousMillimeters() / 10;
          LOG_DEBUG("AVOID: Quick scan RIGHT distance: %d cm", robot.quickScanRightDist);
          robot.actionStarted = false;
          robot.obstacleAvoidanceState = AVOID_EVALUATE_QUICK_SCANS;
        } else if (millis() - robot.lastActionTime > robot.turretMoveTime + LASER_DATA_TIMEOUT_MS) {
          // Timeout waiting for laser data
          LOG_WARN("AVOID: Timeout waiting for RIGHT laser data");
          robot.quickScanRightDist = robot.maxUltrasonicDistance; // Safe fallback
          robot.actionStarted = false;
          robot.obstacleAvoidanceState = AVOID_EVALUATE_QUICK_SCANS;
        }
      }
      break;

    case AVOID_EVALUATE_QUICK_SCANS:
      if (robot.quickScanLeftDist > robot.minDistForValidPath) {
        LOG_DEBUG("AVOID: Quick escape available to the LEFT.");
        float currentHeading = getCalibratedHeading(robot);
        if (!std::isnan(currentHeading)) {
          robot.capCibleRotation = currentHeading - 60; // Turn more than 45 to be safe
        }
        robot.obstacleAvoidanceState = AVOID_TURN_TO_PATH;
      } else if (robot.quickScanRightDist > robot.minDistForValidPath) {
        LOG_DEBUG("AVOID: Quick escape available to the RIGHT.");
        float currentHeading = getCalibratedHeading(robot);
        if (!std::isnan(currentHeading)) {
          robot.capCibleRotation = currentHeading + 60;
        }
        robot.obstacleAvoidanceState = AVOID_TURN_TO_PATH;
      } else {
        LOG_DEBUG("AVOID: No quick escape. Performing full scan.");
        robot.obstacleAvoidanceState = AVOID_PERFORM_FULL_SCAN;
      }
      robot.actionStarted = false;
      break;

    case AVOID_PERFORM_FULL_SCAN:
      robot.bestAvoidAngle = findWidestPath(robot);
      robot.obstacleAvoidanceState = AVOID_EVALUATE_FULL_SCAN;
      break;

    case AVOID_EVALUATE_FULL_SCAN:
      if (robot.bestAvoidAngle != -1) {
        LOG_DEBUG("AVOID: Widest path found at angle %d.", robot.bestAvoidAngle);
        float angleOffset = robot.bestAvoidAngle - 90.0;
        float currentHeading = getCalibratedHeading(robot);
        if (!std::isnan(currentHeading)) {
          robot.capCibleRotation = currentHeading + angleOffset;
        }
        robot.obstacleAvoidanceState = AVOID_TURN_TO_PATH;
      } else {
        LOG_DEBUG("AVOID: No valid path found. Retreating.");
        robot.obstacleAvoidanceState = AVOID_RETREAT_TURN;
      }
      robot.actionStarted = false;
      break;

    case AVOID_TURN_TO_PATH: {
      float error = calculateHeadingError(robot.capCibleRotation, getCalibratedHeading(robot));
      if (abs(error) < robot.turnTolerance) {
        LOG_DEBUG("AVOID: Turn complete. Moving forward.");
        robot.obstacleAvoidanceState = AVOID_MOVE_FORWARD;
        robot.actionStarted = false;
        break;
      }
      int pivotSpeed = robot.speedRotation;
      if (error > 0) { targetA = -pivotSpeed; targetB = pivotSpeed; }
      else { targetA = pivotSpeed; targetB = -pivotSpeed; }
      break;
    }
    
    case AVOID_MOVE_FORWARD:
      if(isObstacleDetected(robot)) {
          LOG_DEBUG("AVOID: Obstacle re-detected while moving forward. Re-initializing avoidance.");
          robot.obstacleAvoidanceState = AVOID_INIT;
          break;
      }
      if (!robot.actionStarted) {
        robot.lastActionTime = millis();
        robot.actionStarted = true;
      }
      if (millis() - robot.lastActionTime < 1500) { // Move forward for 1.5 seconds
        targetA = robot.speedSlow;
        targetB = robot.speedSlow;
      } else {
        LOG_DEBUG("AVOID: Finished escape maneuver. Returning to IDLE.");
        changeState(robot, IDLE);
      }
      break;

    case AVOID_RETREAT_TURN:
        LOG_DEBUG("AVOID: Retreating, turning 180 degrees.");
        robot.capCibleRotation = getCalibratedHeading(robot) + 180;
        robot.obstacleAvoidanceState = AVOID_TURN_TO_PATH; // Reuse the turning state
        break;

    default:
      LOG_ERROR("AVOID: Reached unknown avoidance state! Resetting.");
      robot.obstacleAvoidanceState = AVOID_INIT;
      break;
  }
}

void handleObstacleAvoidanceState(Robot& robot, int& targetA, int& targetB) {
    handleSmartAvoidanceState(robot, targetA, targetB);
}

void updateMotorControl(Robot& robot) {
  if (robot.batteryIsCritical) { Arret(); return; }

  int targetA = 0;
  int targetB = 0;

  switch (robot.currentState) {
    case IDLE: targetA = 0; targetB = 0; break;
    case MOVING_FORWARD: handleMovingForwardState(robot, targetA, targetB); break;
    case FOLLOW_HEADING: handleFollowHeadingState(robot, targetA, targetB); break;
    case MOVING_BACKWARD: targetA = -robot.targetSpeed; targetB = -robot.targetSpeed; break;
    case MANUAL_COMMAND_MODE:
        // Safety override for manual movement in Ackermann steering
        if (bumperPressed || (robot.dusm < 5 && robot.dusm > 0)) {
            targetA = 0; // Prevent all motion
            targetB = 0; // Prevent all motion
            Servodirection.write(robot.servoNeutralDir); // Center steering
            trigger_rumble(40, 200, 0); // Stronger haptic notification for safety override
        } else {
            // If Ackermann steering is enabled, but only a pivot turn is commanded (velocity is 0)
            // then use calculateManualPwm for differential pivot turn.
            if (robot.manualTargetVelocity == 0 && robot.manualTargetTurn != 0) {
                calculateManualPwm(robot, targetA, targetB);
            } else {
                // Otherwise, proceed with Ackermann steering (velocity from left joystick Y-axis).
                targetA = robot.manualTargetVelocity;
                targetB = robot.manualTargetVelocity;
            }
        }
        break;
    case OBSTACLE_AVOIDANCE: 
    case SMART_AVOIDANCE: 
      handleSmartAvoidanceState(robot, targetA, targetB); 
      break;
    // ... other cases
    default: targetA = 0; targetB = 0; break;
  }

  // --- Final Motor Application ---
  robot.currentPwmA = applySpeedRamp(robot, targetA, robot.currentPwmA);
  robot.currentPwmB = applySpeedRamp(robot, targetB, robot.currentPwmB);
  
  int finalA = (int)robot.currentPwmA;
  int finalB = (int)robot.currentPwmB;

  finalB = (int)(finalB * robot.motorBCalibration);
  if (robot.controlInverted) { finalA *= -1; finalB *= -1; }

  if (abs(finalA) < MIN_SPEED_TO_MOVE && targetA == 0) finalA = 0;
  if (abs(finalB) < MIN_SPEED_TO_MOVE && targetB == 0) finalB = 0;

  motorA.motorGo(constrain(finalA, -PWM_MAX, PWM_MAX));
  motorB.motorGo(constrain(finalB, -PWM_MAX, PWM_MAX));
}

void calculateManualPwm(Robot& robot, int& outPwmA, int& outPwmB) {
    int velocity = robot.manualTargetVelocity;
    int turn = robot.manualTargetTurn;

    // Safety override for manual movement
    if (bumperPressed || (robot.dusm < 5 && robot.dusm > 0)) {
        velocity = 0; // Prevent all motion
        turn = 0;     // Stop turning
        Servodirection.write(robot.servoNeutralDir); // Center steering
        trigger_rumble(40, 200, 0); // Stronger haptic notification for safety override
    }

    outPwmA = constrain(velocity - turn, -PWM_MAX, PWM_MAX);
    outPwmB = constrain(velocity + turn, -PWM_MAX, PWM_MAX);
}

float easeOutInSine(float t) { return 0.5 * (1 + sin(PI * (t - 0.5))); }
void handleEmergencyEvasionState(Robot& robot, int& pwmA, int& pwmB) {}
void handleStuckState(Robot& robot, int& pwmA, int& pwmB) {}
void handleHeadAnimation(Robot& robot, bool changeStateOnFinish) {}
void handleAnimatingHeadState(Robot& robot) {}
void handleSentryModeState(Robot& robot) {}
void handleCheckingGroundState(Robot& robot, int& pwmA, int& pwmB) {}
void applyTractionControl(int& pwmA, int& pwmB, Robot& robot) {}
