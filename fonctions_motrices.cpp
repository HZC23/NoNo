#include "fonctions_motrices.h"
#include "hardware.h"
#include "comms.h"
#include "logger.h"

// --- Private Function Prototypes ---
float calculateHeadingError(float target, float current);
bool isObstacleDetected(Robot& robot);
int applySpeedRamp(Robot& robot, float target, float current);
void handleEmergencyEvasionState(Robot& robot, int& pwmA, int& pwmB);
void handleStuckState(Robot& robot, int& pwmA, int& pwmB);
void handleHeadAnimation(Robot& robot, bool changeStateOnFinish);
void handleMovingForwardState(Robot& robot, int& targetA, int& targetB);
void handleFollowHeadingState(Robot& robot, int& targetA, int& targetB);
void handleSmartAvoidanceState(Robot& robot, int& targetA, int& targetB);
void handleSentryModeState(Robot& robot);
void handleCheckingGroundState(Robot& robot, int& pwmA, int& pwmB);
void handleAnimatingHeadState(Robot& robot);
void applyTractionControl(int& pwmA, int& pwmB, Robot& robot);

// --- Function Implementations ---

void changeState(Robot& robot, RobotState newState, ObstacleAvoidanceState avoidState) {
  if (robot.currentState == newState && newState != OBSTACLE_AVOIDANCE) return;

  // --- Servo Power Management ---
  if (newState == IDLE) {
    LOG_DEBUG("Detaching servos for IDLE state.");
    tourelle.detach();
    Servodirection.detach();
  } else if (robot.currentState == IDLE) {
    LOG_DEBUG("Re-attaching servos and setting to neutral.");
    tourelle.attach();
    Servodirection.attach(PINDIRECTION);
    Servodirection.write(robot.servoNeutralDir);
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
    robot.obstacleAvoidanceState = AVOID_IDLE;
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
  targetA = robot.speedAvg;
  targetB = robot.speedAvg;
}

void handleFollowHeadingState(Robot& robot, int& targetA, int& targetB) {
  if (isObstacleDetected(robot)) {
      changeState(robot, SMART_AVOIDANCE);
      return;
  }
  // Simplified
  float error = calculateHeadingError(robot.capCibleRotation, getCalibratedHeading(robot));
  if (abs(error) < robot.turnTolerance) {
      changeState(robot, MOVING_FORWARD);
      return;
  }
  int pivotSpeed = robot.speedRotation;
  if (error > 0) { targetA = -pivotSpeed; targetB = pivotSpeed; }
  else { targetA = pivotSpeed; targetB = -pivotSpeed; }
}

void handleSmartAvoidanceState(Robot& robot, int& targetA, int& targetB) {
  targetA = 0; // Default to stopping
  targetB = 0;
  const int QUICK_SCAN_ANGLE = 45; // 45 degrees left/right of center

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
      if (millis() - robot.lastActionTime > robot.turretMoveTime) {
        robot.quickScanLeftDist = vl53->readRangeContinuousMillimeters() / 10;
        LOG_DEBUG("AVOID: Quick scan LEFT distance: %d cm", robot.quickScanLeftDist);
        robot.actionStarted = false;
        robot.obstacleAvoidanceState = AVOID_QUICK_SCAN_RIGHT;
      }
      break;

    case AVOID_QUICK_SCAN_RIGHT:
      if (!robot.actionStarted) {
        tourelle.write(90 + QUICK_SCAN_ANGLE, robot.servoNeutralTurret);
        robot.lastActionTime = millis();
        robot.actionStarted = true;
      }
      if (millis() - robot.lastActionTime > robot.turretMoveTime) {
        robot.quickScanRightDist = vl53->readRangeContinuousMillimeters() / 10;
        LOG_DEBUG("AVOID: Quick scan RIGHT distance: %d cm", robot.quickScanRightDist);
        robot.actionStarted = false;
        robot.obstacleAvoidanceState = AVOID_EVALUATE_QUICK_SCANS;
      }
      break;

    case AVOID_EVALUATE_QUICK_SCANS:
      if (robot.quickScanLeftDist > robot.minDistForValidPath) {
        LOG_DEBUG("AVOID: Quick escape available to the LEFT.");
        robot.capCibleRotation = getCalibratedHeading(robot) - 60; // Turn more than 45 to be safe
        robot.obstacleAvoidanceState = AVOID_TURN_TO_PATH;
      } else if (robot.quickScanRightDist > robot.minDistForValidPath) {
        LOG_DEBUG("AVOID: Quick escape available to the RIGHT.");
        robot.capCibleRotation = getCalibratedHeading(robot) + 60;
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
        robot.capCibleRotation = getCalibratedHeading(robot) + angleOffset;
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

// All calls to the old avoidance now go to the new one
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
    case MOVING_BACKWARD: targetA = -robot.vitesseCible; targetB = -robot.vitesseCible; break;
    case MANUAL_COMMAND_MODE: calculateManualPwm(robot, targetA, targetB); break;
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
    outPwmA = constrain(velocity - turn, -PWM_MAX, PWM_MAX);
    outPwmB = constrain(velocity + turn, -PWM_MAX, PWM_MAX);
}

// ... other function stubs for brevity
float easeOutInSine(float t) { return 0.5 * (1 + sin(PI * (t - 0.5))); }
void handleEmergencyEvasionState(Robot& robot, int& pwmA, int& pwmB) {}
void handleStuckState(Robot& robot, int& pwmA, int& pwmB) {}
void handleHeadAnimation(Robot& robot, bool changeStateOnFinish) {}
void handleAnimatingHeadState(Robot& robot) {}
void handleSentryModeState(Robot& robot) {}
void handleCheckingGroundState(Robot& robot, int& pwmA, int& pwmB) {}
void applyTractionControl(int& pwmA, int& pwmB, Robot& robot) {}
