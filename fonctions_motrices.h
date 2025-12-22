#ifndef FONCTIONS_MOTRICES_H
#define FONCTIONS_MOTRICES_H

#include "config.h"
#include "state.h"
#include "compass.h" // For getCalibratedHeading
#include "support.h" // For Phare control
#include "turret_control.h" // For advanced turret behaviors

// Function Prototypes
void changeState(Robot& robot, RobotState newState);
void Arret();
void updateMotorControl(Robot& robot);
float calculateHeadingError(float target, float current);
bool isObstacleDetected(Robot& robot);
void handleEmergencyEvasionState(Robot& robot, int& pwmA, int& pwmB);
void handleStuckState(Robot& robot, int& pwmA, int& pwmB);

// --- IMPLEMENTATIONS ---

inline bool isObstacleDetected(Robot& robot) {
    bool ultraSonicObstacle = (robot.dusm < ULTRASONIC_OBSTACLE_THRESHOLD_CM && robot.dusm > 0);
    bool laserObstacle = (robot.laserInitialized && robot.distanceLaser < LASER_OBSTACLE_THRESHOLD_CM && robot.distanceLaser > 0);
    
    robot.obstacleDetectedByLaser = laserObstacle; // Set the flag if laser detects too close

    return ultraSonicObstacle || laserObstacle;
}

// Function to manage the steering servo (Ackermann) and return the angle
inline int setAckermannAngle(int angleError) {
    // Maps the angle error (-30 to +30) to the servo angle (e.g., 45 to 135)
    int servoAngle = map(angleError, -SEUIL_BASCULE_DIRECTION, SEUIL_BASCULE_DIRECTION, SERVO_DIR_MIN, SERVO_DIR_MAX);
    servoAngle = constrain(servoAngle, SERVO_DIR_MIN, SERVO_DIR_MAX);
    Servodirection.write(servoAngle);
    return servoAngle;
}

// New intelligent navigation function
inline void smartNavigation(Robot& robot) {
    // 1. Calculate heading error
    float currentHeading = getCalibratedHeading(robot);
    float error = calculateHeadingError(robot.capCibleRotation, currentHeading);

    // 2. Decide on the mode
    if (abs(error) > SEUIL_BASCULE_DIRECTION) {
        // === PIVOT MODE (Wheel inversion) ===
        Servodirection.write(NEUTRE_DIRECTION); 
        int pivotSpeed = VITESSE_ROTATION; 
        if (error > 0) { // Target is to the right
            motorA.motorGo(-pivotSpeed);
            motorB.motorGo(pivotSpeed);
        } else { // Target is to the left
            motorA.motorGo(pivotSpeed);
            motorB.motorGo(-pivotSpeed);
        }
        
    } else {
        // === ACKERMANN MODE (Smooth driving) ===
        int servoAngle = setAckermannAngle((int)error);
        syncTurretWithSteering(servoAngle); // LOOK-AHEAD: Point turret into the turn
        
        int adjustment = (int)(error * 1.0);
        int pwmLeft = robot.vitesseCible + adjustment;
        int pwmRight = robot.vitesseCible - adjustment;
        
        motorA.motorGo(constrain(pwmLeft, -PWM_MAX, PWM_MAX));
        motorB.motorGo(constrain(pwmRight, -PWM_MAX, PWM_MAX));
    }
}


inline void changeState(Robot& robot, RobotState newState) {
  if (robot.currentState == newState) return;
  
  // Store the current state if we are transitioning to an animation state
  if (newState == ANIMATING_HEAD) {
    robot.stateBeforeHeadAnimation = robot.currentState;
  }
  
  robot.currentState = newState;
  robot.actionStarted = false;
  robot.lastActionTime = millis();
  
  // If returning to IDLE, reset curious look targets to re-center the head.
  if (newState == IDLE) {
    robot.curiousTargetH = SCAN_CENTER_ANGLE;
    robot.curiousTargetV = NEUTRE_TOURELLE;
  }

  if (newState == OBSTACLE_AVOIDANCE) {
    robot.consecutiveAvoidManeuvers++;
    robot.obstacleAvoidanceState = AVOID_START; // Reset the sub-state
  } else if (robot.currentState != OBSTACLE_AVOIDANCE) { // Avoid resetting if we are already in a sub-maneuver
    robot.consecutiveAvoidManeuvers = 0; // Reset counter on other state changes
  }

  // Reset sub-state machines when leaving their primary state
  if (newState != CHECKING_GROUND) {
    robot.groundCheckState = GC_START;
  }
  if (newState != ANIMATING_HEAD) {
    robot.currentHeadAnimation = ANIM_NONE;
  }
  if (newState != EMERGENCY_EVASION) {
    robot.evasionState = EVADE_START;
  }


  if (DEBUG_MODE) {
    Serial.print("Transition d'état -> ");
    Serial.println(newState);
  }
}

inline void Arret() {
  motorA.motorBrake(PWM_MAX);
  motorB.motorBrake(PWM_MAX);
}


// Helper function for smooth sine-wave based easing
inline float easeOutInSine(float t) {
    return 0.5 * (1 + sin(PI * (t - 0.5)));
}

inline void handleHeadAnimation(Robot& robot, bool changeStateOnFinish) {
  if (robot.currentHeadAnimation == ANIM_NONE) return;

  unsigned long elapsed = millis() - robot.headAnimStartTime;

  if (robot.currentHeadAnimation == ANIM_SHAKE_NO) {
    int cycleTime = HEAD_SHAKE_NO_CYCLE_DURATION_MS * 2; // Full back-and-forth cycle
    int totalDuration = robot.headAnimCycles * cycleTime;

    if (elapsed < totalDuration) {
      float progress = (float)(elapsed % cycleTime) / cycleTime;
      float eased_progress = easeOutInSine(progress);
      
      int angle = map(eased_progress, 0, 1, HEAD_SHAKE_NO_ANGLE_EXTREME_RIGHT, HEAD_SHAKE_NO_ANGLE_EXTREME_LEFT);
      tourelle.write(angle, NEUTRE_TOURELLE);
    } else {
      if (changeStateOnFinish) {
        tourelle.write(SCAN_CENTER_ANGLE, NEUTRE_TOURELLE);
        changeState(robot, robot.stateBeforeHeadAnimation);
      } else {
        robot.currentHeadAnimation = ANIM_NONE; // Animation is over
      }
    }
  } else if (robot.currentHeadAnimation == ANIM_NOD_YES) {
    int cycleTime = HEAD_NOD_YES_CYCLE_DURATION_MS * 2; // Full up-and-down cycle
    int totalDuration = robot.headAnimCycles * cycleTime;

    if (elapsed < totalDuration) {
      float progress = (float)(elapsed % cycleTime) / cycleTime;
      float eased_progress = easeOutInSine(progress);

      int angle = map(eased_progress, 0, 1, HEAD_NOD_YES_ANGLE_DOWN, HEAD_NOD_YES_ANGLE_UP);
      tourelle.write(tourelle.getAngleHorizontal(), angle);
    } else {
      if (changeStateOnFinish) {
        tourelle.write(tourelle.getAngleHorizontal(), NEUTRE_TOURELLE);
        changeState(robot, robot.stateBeforeHeadAnimation);
      } else {
        robot.currentHeadAnimation = ANIM_NONE; // Animation is over
      }
    }
  }
}

// --- STATE HANDLERS ---

inline void handleIdleState(Robot& robot, int& pwmA, int& pwmB) {
  pwmA = 0;
  pwmB = 0;
  // Let the main loop's updateTurret() handle idle movement
}

inline void handleMovingForwardState(Robot& robot, int& pwmA, int& pwmB) {
  if (millis() - robot.lastActionTime > STALL_DETECTION_TIMEOUT_MS) {
      if(DEBUG_MODE) Serial.println("STALL detected in MOVING_FORWARD");
      changeState(robot, STUCK);
      return;
  }
  if (millis() - robot.lastCliffCheckTime > CLIFF_CHECK_INTERVAL_MS) {
      robot.stateBeforeGroundCheck = robot.currentState;
      changeState(robot, CHECKING_GROUND);
      return;
  }
  if (robot.currentNavMode == AUTONOMOUS_CONTROL && isObstacleDetected(robot)) {
      changeState(robot, OBSTACLE_AVOIDANCE);
  } else {
      pwmA = robot.vitesseCible;
      pwmB = robot.vitesseCible;
  }
}

inline void handleMovingBackwardState(Robot& robot, int& pwmA, int& pwmB) {
  pwmA = -robot.vitesseCible;
  pwmB = -robot.vitesseCible;
}

inline void handleManualBackwardState(Robot& robot, int& pwmA, int& pwmB) {
  if (isObstacleDetected(robot)) {
    pwmA = 0;
    pwmB = 0;
    changeState(robot, IDLE);
  } else {
    pwmA = -robot.vitesseCible;
    pwmB = -robot.vitesseCible;
  }
}

inline void handleManualForwardState(Robot& robot, int& pwmA, int& pwmB) {
  if (isObstacleDetected(robot)) {
    pwmA = 0;
    pwmB = 0;
    changeState(robot, IDLE);
  } else {
    pwmA = robot.vitesseCible;
    pwmB = robot.vitesseCible;
  }
}

inline void handleManualTurningLeftState(Robot& robot, int& pwmA, int& pwmB) {
  if (isObstacleDetected(robot)) {
    pwmA = 0;
    pwmB = 0;
    changeState(robot, IDLE);
  } else {
    int vitesseRotation = constrain(robot.vitesseCible, 0, VITESSE_ROTATION_MAX);
    pwmA = vitesseRotation;
    pwmB = -vitesseRotation;
  }
}

inline void handleManualTurningRightState(Robot& robot, int& pwmA, int& pwmB) {
  if (isObstacleDetected(robot)) {
    pwmA = 0;
    pwmB = 0;
    changeState(robot, IDLE);
  } else {
    int vitesseRotation = constrain(robot.vitesseCible, 0, VITESSE_ROTATION_MAX);
    pwmA = -vitesseRotation;
    pwmB = vitesseRotation;
  }
}

inline void handleFollowHeadingState(Robot& robot, int& pwmA, int& pwmB) {
  if (millis() - robot.lastActionTime > STALL_DETECTION_TIMEOUT_MS) {
      if(DEBUG_MODE) Serial.println("STALL detected in FOLLOW_HEADING");
      changeState(robot, STUCK);
      return;
  }
  if (millis() - robot.lastCliffCheckTime > CLIFF_CHECK_INTERVAL_MS) {
      robot.stateBeforeGroundCheck = robot.currentState;
      changeState(robot, CHECKING_GROUND);
      return;
  }
  if (isObstacleDetected(robot)) {
      changeState(robot, OBSTACLE_AVOIDANCE);
      return;
  }

  float currentHeading = getCalibratedHeading(robot);
  float error = calculateHeadingError(robot.capCibleRotation, currentHeading);

  if (abs(error) < TOLERANCE_VIRAGE) {
      Arret();
      Servodirection.write(NEUTRE_DIRECTION);
      changeState(robot, IDLE);
      return;
  }

  if (abs(error) > SEUIL_BASCULE_DIRECTION) {
      Servodirection.write(NEUTRE_DIRECTION); 
      int pivotSpeed = VITESSE_ROTATION; 
      if (error > 0) { // Turn Right
          pwmA = -pivotSpeed;
          pwmB = pivotSpeed;
      } else { // Turn Left
          pwmA = pivotSpeed;
          pwmB = -pivotSpeed;
      }
  } else {
      int servoAngle = setAckermannAngle((int)error);
      syncTurretWithSteering(servoAngle); // LOOK-AHEAD
      int adjustment = (int)(error * 1.5);
      
      pwmA = robot.vitesseCible - adjustment; 
      pwmB = robot.vitesseCible + adjustment; 
  }
}

inline void handleMaintainHeadingState(Robot& robot, int& pwmA, int& pwmB) {
  float errorMaintain = calculateHeadingError(robot.Ncap, getCalibratedHeading(robot));
  int adjustment = Kp_HEADING * errorMaintain;
  pwmA = adjustment;
  pwmB = -adjustment;
}

// *** REFACTORED OBSTACLE AVOIDANCE ***
inline void handleObstacleAvoidanceState(Robot& robot, int& pwmA, int& pwmB) {
  switch (robot.obstacleAvoidanceState) {
    case AVOID_START:
      Arret();
      if (DEBUG_MODE) Serial.println("AVOID: Starting scan for clearest path.");
      robot.obstacleAvoidanceState = AVOID_FULL_SCAN_START; // Go directly to scanning
      break;

    case AVOID_FULL_SCAN_START: {
      int bestAngle = findClearestPath(); // This is a blocking call from turret_control.h
      
      if (bestAngle != -1) {
        if (DEBUG_MODE) {
          Serial.print("AVOID: Found clearest path at angle: ");
          Serial.println(bestAngle);
        }
        float currentHeading = getCalibratedHeading(robot);
        // The angle from the turret is relative to the robot's front.
        // 90 degrees is straight ahead.
        float angleOffset = bestAngle - 90.0;
        
        robot.capCibleRotation = currentHeading + angleOffset;
        // Normalize heading
        if (robot.capCibleRotation < 0) robot.capCibleRotation += 360;
        if (robot.capCibleRotation >= 360) robot.capCibleRotation -= 360;
        
        robot.vitesseCible = VITESSE_MOYENNE;
        if (DEBUG_MODE) {
          Serial.print("AVOID: New target heading: ");
          Serial.println(robot.capCibleRotation);
        }
        changeState(robot, FOLLOW_HEADING);

      } else {
        // No path found, must be completely blocked.
        if (DEBUG_MODE) Serial.println("AVOID: No clear path found. Backing up.");
        robot.obstacleAvoidanceState = AVOID_BACKUP;
      }
      break;
    }

    case AVOID_BACKUP:
      if (!robot.actionStarted) {
        robot.lastActionTime = millis();
        robot.actionStarted = true;
      }

      if (millis() - robot.lastActionTime < AVOID_BACKUP_DURATION_MS) {
        pwmA = -robot.speedSlow;
        pwmB = -robot.speedSlow;
      } else {
        if (robot.consecutiveAvoidManeuvers >= MAX_CONSECUTIVE_AVOID_MANEUVERS) {
          if (DEBUG_MODE) Serial.println("AVOID: Max consecutive maneuvers reached. Giving up.");
          robot.stateBeforeHeadAnimation = IDLE;
          robot.currentHeadAnimation = ANIM_SHAKE_NO;
          robot.headAnimStartTime = millis();
          robot.headAnimCycles = 3;
          changeState(robot, ANIMATING_HEAD);
        } else {
          changeState(robot, OBSTACLE_AVOIDANCE); // Try scanning again
        }
      }
      break;

    default:
      // In case of unknown sub-state, reset the avoidance process.
      robot.obstacleAvoidanceState = AVOID_START;
      break;
  }
}


inline void handleSmartAvoidanceState(Robot& robot, int& pwmA, int& pwmB) {
  if (millis() - robot.lastCliffCheckTime > CLIFF_CHECK_INTERVAL_MS) {
      robot.stateBeforeGroundCheck = robot.currentState;
      changeState(robot, CHECKING_GROUND);
      return;
  }
  if (isObstacleDetected(robot)) { // More cautious distance for this mode
    changeState(robot, OBSTACLE_AVOIDANCE);
  } else {
    // Move forward, following the current Ncap
    robot.vitesseCible = VITESSE_MOYENNE;
    float errorFollow = calculateHeadingError(robot.Ncap, getCalibratedHeading(robot));
    int adjustment = Kp_HEADING * errorFollow;
    pwmA = robot.vitesseCible + adjustment;
    pwmB = robot.vitesseCible - adjustment;
  }
}

inline void handleSentryModeState(Robot& robot) {
  switch(robot.sentryState) {
    case SENTRY_IDLE: {
      bool pirState = digitalRead(PIR);
      if (pirState == HIGH && robot.lastPIRState == LOW) {
        if (DEBUG_MODE) Serial.println("SENTRY: PIR Triggered! Starting scan.");
        robot.sentryState = SENTRY_SCAN_START;
      }
      robot.lastPIRState = pirState;
      break;
    }

    case SENTRY_SCAN_START:
      robot.currentScanAngleH = SCAN_H_START_ANGLE;
      tourelle.write(robot.currentScanAngleH, NEUTRE_TOURELLE - robot.currentPitch);
      robot.lastActionTime = millis();
      robot.sentryState = SENTRY_SCAN_STEP;
      break;

    case SENTRY_SCAN_STEP:
      if (millis() - robot.lastActionTime > SENTRY_SCAN_SPEED_MS) {
        robot.lastActionTime = millis();
        
        if (robot.distanceLaser > 0 && robot.distanceLaser < SENTRY_DETECTION_RANGE_CM) {
          if (DEBUG_MODE) {
            Serial.print("SENTRY: Intruder detected at angle ");
            Serial.print(robot.currentScanAngleH);
            Serial.print(" and distance ");
            Serial.println(robot.distanceLaser);
          }
          robot.intruderAngle = robot.currentScanAngleH;
          robot.sentryState = SENTRY_TRACKING;
          robot.actionStarted = false; // Reset for tracking state
          return;
        }

        robot.currentScanAngleH += SCAN_H_STEP;
        if (robot.currentScanAngleH > SCAN_H_END_ANGLE) {
          if (DEBUG_MODE) Serial.println("SENTRY: Scan complete, no intruder found.");
          tourelle.write(SCAN_CENTER_ANGLE, NEUTRE_TOURELLE - robot.currentPitch);
          robot.sentryState = SENTRY_IDLE; // False alarm, go back to idle
        } else {
          tourelle.write(robot.currentScanAngleH, NEUTRE_TOURELLE - robot.currentPitch);
        }
      }
      break;

    case SENTRY_TRACKING:
      if (!robot.actionStarted) {
        robot.actionStarted = true;
        tourelle.write(robot.intruderAngle, NEUTRE_TOURELLE - robot.currentPitch);
        PhareAllume();
        robot.lastActionTime = millis();
        if (DEBUG_MODE) Serial.println("SENTRY: Tracking intruder.");
      }
      break;
  }
}

inline void handleCheckingGroundState(Robot& robot, int& pwmA, int& pwmB) {
  // Keep moving forward while checking for cliffs
  pwmA = robot.vitesseCible;
  pwmB = robot.vitesseCible;
  switch(robot.groundCheckState) {
    case GC_START:
      // Arret(); // Don't stop motors for a stable measurement, check while moving
      robot.groundCheckState = GC_LOOK_DOWN;
      break;
    case GC_LOOK_DOWN:
      tourelle.write(tourelle.getAngleHorizontal(), ANGLE_SOL);
      robot.turretMoveStartTime = millis();
      robot.groundCheckState = GC_WAIT;
      break;
    case GC_WAIT:
      if (millis() - robot.turretMoveStartTime > TURRET_MOVE_TIME_MS) {
        robot.groundCheckState = GC_CHECK;
      }
      break;
    case GC_CHECK:
      // Distance Laser is updated in the main loop
      if (robot.distanceLaser > SEUIL_VIDE || robot.distanceLaser == 0) { // distance 0 can be an error/out of range
        changeState(robot, CLIFF_DETECTED);
      } else {
        robot.groundCheckState = GC_LOOK_UP;
      }
      break;
    case GC_LOOK_UP:
      tourelle.write(tourelle.getAngleHorizontal(), NEUTRE_TOURELLE - robot.currentPitch);
      robot.turretMoveStartTime = millis();
      robot.groundCheckState = GC_FINISH;
      break;
    case GC_FINISH:
      if (millis() - robot.turretMoveStartTime > TURRET_MOVE_TIME_MS) {
        robot.lastCliffCheckTime = millis();
        changeState(robot, robot.stateBeforeGroundCheck); // Return to previous state
      }
      break;
  }
}

inline void handleCliffDetectedState(Robot& robot, int& pwmA, int& pwmB) {
  // Emergency maneuver
  setLcdText(robot, "ATTENTION VIDE !");
  if (!robot.actionStarted) {
      robot.actionStarted = true;
      robot.lastActionTime = millis();
      Arret();
  }
  if (millis() - robot.lastActionTime < AVOID_BACKUP_DURATION_MS) {
      pwmA = -robot.speedSlow;
      pwmB = -robot.speedSlow;
  } else {
      changeState(robot, IDLE);
  }
}

inline void handleAnimatingHeadState(Robot& robot) {
  // Stop motors during animation
  motorA.motorGo(0);
  motorB.motorGo(0);
  handleHeadAnimation(robot, true);
}


inline void updateMotorControl(Robot& robot) {
  // --- CRITICAL BATTERY E-STOP ---
  if (robot.batteryIsCritical) {
      Arret();
      return;
  }

  int pwmA = 0;
  int pwmB = 0;

  switch (robot.currentState) {
    case IDLE:
      handleIdleState(robot, pwmA, pwmB);
      break;
    case MOVING_FORWARD:
      handleMovingForwardState(robot, pwmA, pwmB);
      break;
    case MOVING_BACKWARD:
      handleMovingBackwardState(robot, pwmA, pwmB);
      break;
    case MANUAL_BACKWARD:
      handleManualBackwardState(robot, pwmA, pwmB);
      break;
    case MANUAL_FORWARD:
      handleManualForwardState(robot, pwmA, pwmB);
      break;
    case MANUAL_TURNING_LEFT:
      handleManualTurningLeftState(robot, pwmA, pwmB);
      break;
    case MANUAL_TURNING_RIGHT:
      handleManualTurningRightState(robot, pwmA, pwmB);
      break;
    case FOLLOW_HEADING:
      handleFollowHeadingState(robot, pwmA, pwmB);
      break;
    case MAINTAIN_HEADING:
      handleMaintainHeadingState(robot, pwmA, pwmB);
      break;
    case OBSTACLE_AVOIDANCE:
      handleObstacleAvoidanceState(robot, pwmA, pwmB);
      break;
    case SMART_AVOIDANCE:
      handleSmartAvoidanceState(robot, pwmA, pwmB);
      break;
    case SENTRY_MODE:
      handleSentryModeState(robot);
      break;
    case CHECKING_GROUND:
      handleCheckingGroundState(robot, pwmA, pwmB);
      break;
    case CLIFF_DETECTED:
      handleCliffDetectedState(robot, pwmA, pwmB);
      break;
    case ANIMATING_HEAD:
      handleAnimatingHeadState(robot);
      break;
    case EMERGENCY_EVASION:
      handleEmergencyEvasionState(robot, pwmA, pwmB);
      break;
    case STUCK:
      handleStuckState(robot, pwmA, pwmB);
      break;
    case APP_CONTROL:
      // Motor commands are sent directly from terminal.h
      break;
    default:
      pwmA = 0;
      pwmB = 0;
      break;
  }
  
  pwmB = (int)(pwmB * CALIBRATION_MOTEUR_B);

  // Apply control inversion if enabled
  if (robot.controlInverted) {
    pwmA *= -1;
    pwmB *= -1;
  }
  
  motorA.motorGo(constrain(pwmA, -PWM_MAX, PWM_MAX));
  motorB.motorGo(constrain(pwmB, -PWM_MAX, PWM_MAX));
}

inline void handleEmergencyEvasionState(Robot& robot, int& pwmA, int& pwmB) {
  // "Nono-Escape" maneuver
  switch(robot.evasionState) {
    case EVADE_START:
      Arret();
      // Play a sound, blink lights, etc.
      // lcd.setRGB(255, 0, 0);
      // setLcdText(robot, "Bumper Hit!");
      robot.actionStarted = false;
      robot.evasionState = EVADE_BACKUP;
      break;

    case EVADE_BACKUP:
      if (!robot.actionStarted) {
        robot.actionStarted = true;
        robot.lastActionTime = millis();
        if (DEBUG_MODE) Serial.println("EVADE: Backing up...");
      }

      if (millis() - robot.lastActionTime < 800) { // Back up for 800ms
        pwmA = -VITESSE_MOYENNE;
        pwmB = -VITESSE_MOYENNE;
      } else {
        Arret();
        robot.actionStarted = false;
        robot.evasionState = EVADE_PIVOT;
      }
      break;

    case EVADE_PIVOT: { // Added scope for error variable
      if (!robot.actionStarted) {
        // Here you could add a laser scan to find the best escape direction.
        // For now, we pivot 90 degrees right.
        if (DEBUG_MODE) Serial.println("EVADE: Pivoting...");
        robot.capCibleRotation = getCalibratedHeading(robot) + 90.0;
        if (robot.capCibleRotation >= 360) robot.capCibleRotation -= 360;
        robot.actionStarted = true;
      }
      
      float error = calculateHeadingError(robot.capCibleRotation, getCalibratedHeading(robot));
      if (abs(error) > TOLERANCE_VIRAGE) {
        pwmA = -VITESSE_ROTATION; // Pivot right
        pwmB = VITESSE_ROTATION;
      } else {
        robot.evasionState = EVADE_FINISH;
      }
      break;
    } // End of scope for error variable

    case EVADE_FINISH:
      if (DEBUG_MODE) Serial.println("EVADE: Maneuver complete.");
      Arret();
      robot.evasionState = EVADE_START; // Reset for next time
      changeState(robot, OBSTACLE_AVOIDANCE); // After evading, assess the situation
      break;
  }
}

inline void handleStuckState(Robot& robot, int& pwmA, int& pwmB) {
  // A simplified version of the evasion maneuver
  if (!robot.actionStarted) {
    robot.actionStarted = true;
    robot.lastActionTime = millis();
    if (DEBUG_MODE) Serial.println("STUCK: Attempting to un-stuck...");
    // setLcdText(robot, "I'm stuck!");
    Arret();
  }

  // Back up, then let the standard obstacle avoidance take over.
  if (millis() - robot.lastActionTime < 1000) {
    pwmA = -VITESSE_MOYENNE;
    pwmB = -VITESSE_MOYENNE;
  } else {
    changeState(robot, OBSTACLE_AVOIDANCE);
  }
}

inline float calculateHeadingError(float target, float current) {
  float error = target - current;
  if (error > 180) error -= 360;
  else if (error < -180) error += 360;
  return error;
}

#endif // FONCTIONS_MOTRICES_H