#include "fonctions_motrices.h"
#include "hardware.h"
#include "comms.h"

// --- Private Function Prototypes ---
float calculateHeadingError(float target, float current);
bool isObstacleDetected(Robot& robot);
int applySpeedRamp(Robot& robot, float target, float current);
int getDynamicMaxSteering(Robot& robot, int currentSpeed);
void calculateDifferentialDrive(Robot& robot, int targetSpeed, int steeringAngleRelative, int& outPwmA, int& outPwmB);
void handleEmergencyEvasionState(Robot& robot, int& pwmA, int& pwmB);
void handleStuckState(Robot& robot, int& pwmA, int& pwmB);
void handleHeadAnimation(Robot& robot, bool changeStateOnFinish);
void handleMovingForwardState(Robot& robot, int& targetA, int& targetB);
void handleFollowHeadingState(Robot& robot, int& targetA, int& targetB);
void handleObstacleAvoidanceState(Robot& robot, int& targetA, int& targetB);
void handleSmartAvoidanceState(Robot& robot, int& targetA, int& targetB);
void handleSentryModeState(Robot& robot);
void handleCheckingGroundState(Robot& robot, int& pwmA, int& pwmB);
void handleAnimatingHeadState(Robot& robot);
void applyTractionControl(int& pwmA, int& pwmB, Robot& robot);


// --- Function Implementations ---

void changeState(Robot& robot, RobotState newState) {
  if (robot.currentState == newState) return;
  
  if (newState == ANIMATING_HEAD) {
    robot.stateBeforeHeadAnimation = robot.currentState;
  }
  
  robot.currentState = newState;
  robot.actionStarted = false;
  robot.lastActionTime = millis();
  
  if (newState == IDLE) {
    Arret(); // Hard stop motors
    robot.currentPwmA = 0;
    robot.currentPwmB = 0;
    robot.curiousTargetH = robot.servoNeutralTurret;
    robot.curiousTargetV = robot.servoNeutralTurret;
  }

  if (newState == OBSTACLE_AVOIDANCE) {
    robot.consecutiveAvoidManeuvers++;
    robot.obstacleAvoidanceState = AVOID_START;
  } else if (robot.currentState != OBSTACLE_AVOIDANCE) {
    robot.consecutiveAvoidManeuvers = 0;
  }

  if (newState != CHECKING_GROUND) {
    robot.groundCheckState = GC_START;
  }
  if (newState != ANIMATING_HEAD) {
    robot.currentHeadAnimation = ANIM_NONE;
  }
  if (newState == STUCK) {
      robot.isStuckConfirmed = true;
  } else if (robot.currentState == STUCK && newState != STUCK) {
      robot.isStuckConfirmed = false;
  }
  if (newState != EMERGENCY_EVASION) { robot.evasionState = EVADE_START; }

  if (DEBUG_MODE) { 
    Serial.print("State -> "); 
    Serial.println(stateToString(newState));
  }
}

int applySpeedRamp(Robot& robot, float target, float current) {
    if (abs(target - current) < 1.0) return (int)target;
    return (int)(current + (target - current) * robot.accelRate);
}

int getDynamicMaxSteering(Robot& robot, int currentSpeed) {
    if (currentSpeed < 100) return robot.pivotAngleThreshold;
    float ratio = 1.0 - ((float)(currentSpeed - 100) / (255.0 - 100.0) * 0.4); 
    return (int)(robot.pivotAngleThreshold * ratio);
}

void calculateDifferentialDrive(Robot& robot, int targetSpeed, int steeringAngleRelative, int& outPwmA, int& outPwmB) {
    if (targetSpeed == 0) {
        outPwmA = 0;
        outPwmB = 0;
        return;
    }
    float turnFactor = (float)steeringAngleRelative / (float)robot.pivotAngleThreshold; 
    turnFactor = constrain(turnFactor, -1.0, 1.0);
    float diff_coeff = robot.fwdDiffCoeff;

    if (turnFactor > 0) { // Turn RIGHT
        outPwmA = targetSpeed;
        outPwmB = targetSpeed * (1.0 - (turnFactor * diff_coeff)); 
    } else { // Turn LEFT
        outPwmA = targetSpeed * (1.0 - (abs(turnFactor) * diff_coeff));
        outPwmB = targetSpeed;
    }

    if (targetSpeed != 0 && abs(turnFactor) < 0.8) {
        outPwmA = max(outPwmA, robot.minSpeedToMove);
        outPwmB = max(outPwmB, robot.minSpeedToMove);
    }
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
  if (detectImpactOrStall(robot)) { changeState(robot, STUCK); return; }
  if (millis() - robot.lastActionTime > STALL_DETECTION_TIMEOUT_MS) { changeState(robot, STUCK); return; }
  if (millis() - robot.lastCliffCheckTime > CLIFF_CHECK_INTERVAL_MS) {
      robot.stateBeforeGroundCheck = robot.currentState;
      changeState(robot, CHECKING_GROUND);
      return;
  }
  if (robot.currentNavMode == AUTONOMOUS_CONTROL && isObstacleDetected(robot)) {
      changeState(robot, OBSTACLE_AVOIDANCE);
  } else {
      Servodirection.write(robot.servoNeutralDir);
      tourelle.write(robot.servoNeutralTurret, robot.servoNeutralTurret);
      targetA = robot.speedAvg;
      targetB = robot.speedAvg;
  }
}

void handleFollowHeadingState(Robot& robot, int& targetA, int& targetB) {
  if (detectImpactOrStall(robot)) { changeState(robot, STUCK); return; }
  if (isObstacleDetected(robot)) { changeState(robot, OBSTACLE_AVOIDANCE); return; }

  float currentHeading = getCalibratedHeading(robot);
  float error = calculateHeadingError(robot.capCibleRotation, currentHeading);

  if (abs(error) < robot.turnTolerance) {
      changeState(robot, MOVING_FORWARD); 
      return;
  }

  if (abs(error) > robot.pivotAngleThreshold) {
      Servodirection.write(robot.servoNeutralDir); 
      int pivotSpeed = robot.speedRotation;
      if (error > 0) { targetA = -pivotSpeed; targetB = pivotSpeed; }
      else { targetA = pivotSpeed; targetB = -pivotSpeed; }
  } else {
      setAckermannAngle(robot, (int)error, robot.speedAvg);
      syncTurretWithSteering(Servodirection.read());
      calculateDifferentialDrive(robot, robot.speedAvg, (int)error, targetA, targetB);
      robot.currentSteeringBias = (int)error;
  }
}

void handleObstacleAvoidanceState(Robot& robot, int& targetA, int& targetB) {
  switch (robot.obstacleAvoidanceState) {
    case AVOID_START: Arret(); robot.obstacleAvoidanceState = AVOID_FULL_SCAN_START; break;
    case AVOID_FULL_SCAN_START: {
      int bestAngle = findClearestPath(robot);
      if (bestAngle != -1) {
        float angleOffset = bestAngle - 90.0; 
        robot.capCibleRotation = getCalibratedHeading(robot) + angleOffset;
        if (robot.capCibleRotation < 0) robot.capCibleRotation += 360;
        if (robot.capCibleRotation >= 360) robot.capCibleRotation -= 360;
        changeState(robot, FOLLOW_HEADING);
      } else { robot.obstacleAvoidanceState = AVOID_BACKUP; }
      break;
    }
    case AVOID_BACKUP:
      if (!robot.actionStarted) { robot.lastActionTime = millis(); robot.actionStarted = true; tourelle.write(robot.servoNeutralTurret, robot.servoAngleHeadDown); }
      if (millis() - robot.lastActionTime < robot.avoidBackupDuration) { targetA = -robot.speedSlow; targetB = -robot.speedSlow; } 
      else { 
          if (robot.consecutiveAvoidManeuvers >= 3) {
            robot.consecutiveAvoidManeuvers = 0; 
            float randomTurn = random(120, 240);
            robot.capCibleRotation = getCalibratedHeading(robot) + randomTurn;
            if(robot.capCibleRotation >= 360) robot.capCibleRotation -= 360;
            changeState(robot, FOLLOW_HEADING);
          } else { changeState(robot, OBSTACLE_AVOIDANCE); }
      }
      break;
    default: robot.obstacleAvoidanceState = AVOID_START; break;
  }
}

void handleSmartAvoidanceState(Robot& robot, int& targetA, int& targetB) {
    // ...
}

void handleSentryModeState(Robot& robot) {
    // ...
}

void handleCheckingGroundState(Robot& robot, int& pwmA, int& pwmB) {
    // ...
}

float easeOutInSine(float t) {
    return 0.5 * (1 + sin(PI * (t - 0.5)));
}

void handleHeadAnimation(Robot& robot, bool changeStateOnFinish) {
    // ...
}

void handleAnimatingHeadState(Robot& robot) {
  motorA.motorGo(0);
  motorB.motorGo(0);
  handleHeadAnimation(robot, true);
}

void handleEmergencyEvasionState(Robot& robot, int& pwmA, int& pwmB) {
    // ...
}

void handleStuckState(Robot& robot, int& pwmA, int& pwmB) {
    // ...
}

void applyTractionControl(int& pwmA, int& pwmB, Robot& robot) {
    if (robot.isStuckConfirmed) {
        pwmA *= 0.6;
        pwmB *= 0.6;
    }
}

void updateMotorControl(Robot& robot) {
  if (robot.batteryIsCritical) { Arret(); return; }

  int targetA = 0;
  int targetB = 0;

  switch (robot.currentState) {
    case IDLE: targetA = 0; targetB = 0; break;
    case MOVING_FORWARD: handleMovingForwardState(robot, targetA, targetB); break;
    case SMART_AVOIDANCE: handleSmartAvoidanceState(robot, targetA, targetB); break;
    case FOLLOW_HEADING: handleFollowHeadingState(robot, targetA, targetB); break;
    case MOVING_BACKWARD: targetA = -robot.vitesseCible; targetB = -robot.vitesseCible; break;
    case MANUAL_BACKWARD: targetA = -robot.vitesseCible; targetB = -robot.vitesseCible; break;
    case MANUAL_FORWARD: targetA = robot.vitesseCible; targetB = robot.vitesseCible; break;
    case MANUAL_TURNING_LEFT: targetA = robot.speedRotation; targetB = -robot.speedRotation; break;
    case MANUAL_TURNING_RIGHT: targetA = -robot.speedRotation; targetB = robot.speedRotation; break;
    case OBSTACLE_AVOIDANCE: handleObstacleAvoidanceState(robot, targetA, targetB); break;
    case EMERGENCY_EVASION: handleEmergencyEvasionState(robot, targetA, targetB); break;
    case STUCK: handleStuckState(robot, targetA, targetB); break;
    case CHECKING_GROUND: handleCheckingGroundState(robot, targetA, targetB); break;
    case CLIFF_DETECTED: 
         if (!robot.actionStarted) { robot.actionStarted = true; robot.lastActionTime = millis(); targetA=0; targetB=0; }
         else if (millis() - robot.lastActionTime < robot.avoidBackupDuration) { targetA = -robot.speedSlow; targetB = -robot.speedSlow; } 
         else { changeState(robot, IDLE); }
         break;
    case SENTRY_MODE: handleSentryModeState(robot); break;
    case ANIMATING_HEAD: handleAnimatingHeadState(robot); break;
    case CALIBRATING_COMPASS: calibrateCompass(robot); break;
    case GAMEPAD_CONTROL: 
        targetA = motorA.getPWM(); 
        targetB = motorB.getPWM(); 
        break;
    default: targetA = 0; targetB = 0; break;
  }

  if (abs(robot.currentSteeringBias) > 15) { 
      targetA *= 1.15; 
      targetB *= 1.15;
  }

  if (robot.currentState == EMERGENCY_EVASION || robot.currentState == STUCK || robot.currentState == CLIFF_DETECTED) {
      robot.currentPwmA = targetA;
      robot.currentPwmB = targetB;
  } else {
      robot.currentPwmA = applySpeedRamp(robot, targetA, robot.currentPwmA);
      robot.currentPwmB = applySpeedRamp(robot, targetB, robot.currentPwmB);
  }

  int finalA = (int)robot.currentPwmA;
  int finalB = (int)robot.currentPwmB;

  finalB = (int)(finalB * robot.motorBCalibration);
  if (robot.controlInverted) { finalA *= -1; finalB *= -1; }

  if (abs(finalA) < MIN_SPEED_TO_MOVE && targetA == 0) finalA = 0;
  if (abs(finalB) < MIN_SPEED_TO_MOVE && targetB == 0) finalB = 0;

  applyTractionControl(finalA, finalB, robot);

  motorA.motorGo(constrain(finalA, -PWM_MAX, PWM_MAX));
  motorB.motorGo(constrain(finalB, -PWM_MAX, PWM_MAX));
}