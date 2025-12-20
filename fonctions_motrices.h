#ifndef FONCTIONS_MOTRICES_H
#define FONCTIONS_MOTRICES_H

#include "config.h"
#include "state.h"
#include "compass.h" // For getCalibratedHeading
#include "support.h" // For Phare control

// Function Prototypes
void changeState(Robot& robot, RobotState newState);
void Arret();
void updateMotorControl(Robot& robot);
float calculateHeadingError(float target, float current);
bool isObstacleDetected(Robot& robot);

// --- IMPLEMENTATIONS ---

inline bool isObstacleDetected(Robot& robot) {
    bool ultraSonicObstacle = (robot.dusm < ULTRASONIC_OBSTACLE_THRESHOLD_CM && robot.dusm > 0);
    bool laserObstacle = (robot.laserInitialized && robot.distanceLaser < LASER_OBSTACLE_THRESHOLD_CM && robot.distanceLaser > 0);
    
    robot.obstacleDetectedByLaser = laserObstacle; // Set the flag if laser detects too close

    return ultraSonicObstacle || laserObstacle;
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
  
  if (newState == OBSTACLE_AVOIDANCE) {
    robot.consecutiveAvoidManeuvers++;
    robot.obstacleAvoidanceState = AVOID_START; // Reset the sub-state
  } else {
    robot.consecutiveAvoidManeuvers = 0; // Reset counter on other state changes
  }

  // Reset sub-state machines when leaving their primary state
  if (newState != CHECKING_GROUND) {
    robot.groundCheckState = GC_START;
  }
  if (newState != ANIMATING_HEAD) {
    robot.currentHeadAnimation = ANIM_NONE;
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

// This function was moved from NoNo.ino to be available for updateMotorControl
inline void handleScanning(Robot& robot) {
  if (!robot.actionStarted) {
    robot.currentScanAngleH = SCAN_H_START_ANGLE;
    tourelle.write(robot.currentScanAngleH, NEUTRE_TOURELLE); // Start scan at 90 deg vertical
    robot.lastScanTime = millis();
    robot.actionStarted = true;
    if (DEBUG_MODE) Serial.println(F("Starting unified scan."));
  }

  if (millis() - robot.lastScanTime >= SCAN_DELAY_MS) {
    robot.lastScanTime = millis();
    
    // Report current angle and distance
    if (DEBUG_MODE) {
      Serial.print(F("Scan: Angle="));
      Serial.print(robot.currentScanAngleH);
      Serial.print(F(", Dist="));
      Serial.println(robot.distanceLaser);
    }

    robot.currentScanAngleH += SCAN_H_STEP;
    if (robot.currentScanAngleH > SCAN_H_END_ANGLE) {
      tourelle.write(SCAN_CENTER_ANGLE, NEUTRE_TOURELLE); // Center turret when done
      changeState(robot, IDLE); // Scan complete
      if (DEBUG_MODE) Serial.println(F("Scan complete."));
    } else {
      tourelle.write(robot.currentScanAngleH, NEUTRE_TOURELLE);
    }
  }
}

inline void handleHeadAnimation(Robot& robot, bool changeStateOnFinish) {
  if (robot.currentHeadAnimation == ANIM_NONE) return;

  unsigned long elapsed = millis() - robot.headAnimStartTime;

  if (robot.currentHeadAnimation == ANIM_SHAKE_NO) {
    int cycleTime = HEAD_SHAKE_NO_CYCLE_DURATION_MS;
    int totalDuration = robot.headAnimCycles * cycleTime;

    if (elapsed < totalDuration) {
      int currentCycleTime = elapsed % cycleTime;
      int angle;
      if (currentCycleTime < cycleTime / 2) {
        angle = map(currentCycleTime, 0, cycleTime / 2, HEAD_SHAKE_NO_ANGLE_EXTREME_RIGHT, HEAD_SHAKE_NO_ANGLE_EXTREME_LEFT);
      } else {
        angle = map(currentCycleTime, cycleTime / 2, cycleTime, HEAD_SHAKE_NO_ANGLE_EXTREME_LEFT, HEAD_SHAKE_NO_ANGLE_EXTREME_RIGHT);
      }
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
    int cycleTime = HEAD_NOD_YES_CYCLE_DURATION_MS;
    int totalDuration = robot.headAnimCycles * cycleTime;

    if (elapsed < totalDuration) {
      int currentCycleTime = elapsed % cycleTime;
      int angle;
      if (currentCycleTime < cycleTime / 2) {
        angle = map(currentCycleTime, 0, cycleTime / 2, HEAD_NOD_YES_ANGLE_DOWN, HEAD_NOD_YES_ANGLE_UP);
      } else {
        angle = map(currentCycleTime, cycleTime / 2, cycleTime, HEAD_NOD_YES_ANGLE_UP, HEAD_NOD_YES_ANGLE_DOWN);
      }
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
  if (robot.batteryIsLow) {
    tourelle.write(SCAN_CENTER_ANGLE, ANGLE_TETE_BASSE);
  } else {
    tourelle.write(SCAN_CENTER_ANGLE, NEUTRE_TOURELLE - robot.currentPitch);
  }
  pwmA = 0;
  pwmB = 0;

  // Curious behavior: if idle for a while, look around randomly
  if (millis() - robot.lastActionTime > CURIOUS_MODE_DELAY_MS && 
      millis() - robot.customMessageSetTime > CUSTOM_MESSAGE_DURATION_MS) {
    
    int randomH = random(SCAN_H_START_ANGLE, SCAN_H_END_ANGLE);
    int randomV = random(SCAN_V_START_ANGLE, SCAN_V_END_ANGLE);
    tourelle.write(randomH, randomV);
    robot.lastActionTime = millis(); // Reset timer for next curious movement
    if (DEBUG_MODE) {
        Serial.print("Curious: moved head to H:");
        Serial.print(randomH);
        Serial.print(", V:");
        Serial.println(randomV);
    }
  }
}

inline void handleMovingForwardState(Robot& robot, int& pwmA, int& pwmB) {
  if (millis() - robot.lastCliffCheckTime > CLIFF_CHECK_INTERVAL_MS) {
      robot.stateBeforeGroundCheck = robot.currentState;
      changeState(robot, CHECKING_GROUND);
      return;
  }
  if (robot.currentNavMode == AUTONOMOUS_CONTROL && isObstacleDetected(robot)) { // Obstacle detected in autonomous mode
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
    changeState(robot, IDLE); // Stop and go to IDLE if obstacle detected
  } else {
    pwmA = -robot.vitesseCible;
    pwmB = -robot.vitesseCible;
  }
}

inline void handleManualForwardState(Robot& robot, int& pwmA, int& pwmB) {
  if (isObstacleDetected(robot)) {
    pwmA = 0;
    pwmB = 0;
    changeState(robot, IDLE); // Stop and go to IDLE if obstacle detected
  } else {
    pwmA = robot.vitesseCible;
    pwmB = robot.vitesseCible;
  }
}

inline void handleTurningLeftState(Robot& robot, int& pwmA, int& pwmB) {
  if (!robot.actionStarted) {
    tourelle.write(TURNING_LOOK_LEFT_ANGLE, NEUTRE_TOURELLE); // Regarde à gauche
    robot.lastActionTime = millis();
    robot.actionStarted = true;
  }

  // Timeout after 5 seconds of turning
  if (millis() - robot.lastActionTime > TURNING_TIMEOUT_MS) {
    if (DEBUG_MODE) Serial.println("TURNING_LEFT: Timeout");
    tourelle.write(SCAN_CENTER_ANGLE, NEUTRE_TOURELLE - robot.currentPitch); // Recentrer
    changeState(robot, MOVING_FORWARD);
    return;
  }

  int vitesseRotation = constrain(robot.vitesseCible, 0, VITESSE_ROTATION_MAX);
  pwmA = vitesseRotation;
  pwmB = -vitesseRotation;

  float error = calculateHeadingError(robot.capCibleRotation, getCalibratedHeading(robot));
  if (abs(error) < TOLERANCE_VIRAGE) {
      tourelle.write(SCAN_CENTER_ANGLE, NEUTRE_TOURELLE - robot.currentPitch); // Recentrer
      changeState(robot, MOVING_FORWARD);
  }
}

inline void handleManualTurningLeftState(Robot& robot, int& pwmA, int& pwmB) {
  if (isObstacleDetected(robot)) {
    pwmA = 0;
    pwmB = 0;
    changeState(robot, IDLE); // Stop and go to IDLE if obstacle detected
  } else {
    int vitesseRotation = constrain(robot.vitesseCible, 0, VITESSE_ROTATION_MAX);
    pwmA = vitesseRotation;
    pwmB = -vitesseRotation;
  }
}

inline void handleTurningRightState(Robot& robot, int& pwmA, int& pwmB) {
  if (!robot.actionStarted) {
    tourelle.write(TURNING_LOOK_RIGHT_ANGLE, NEUTRE_TOURELLE); // Regarde à droite
    robot.lastActionTime = millis();
    robot.actionStarted = true;
  }

  // Timeout after 5 seconds of turning
  if (millis() - robot.lastActionTime > TURNING_TIMEOUT_MS) {
    if (DEBUG_MODE) Serial.println("TURNING_RIGHT: Timeout");
    tourelle.write(SCAN_CENTER_ANGLE, NEUTRE_TOURELLE - robot.currentPitch); // Recentrer
    changeState(robot, MOVING_FORWARD);
    return;
  }

  int vitesseRotation = constrain(robot.vitesseCible, 0, VITESSE_ROTATION_MAX);
  pwmA = -vitesseRotation;
  pwmB = vitesseRotation;

  float error = calculateHeadingError(robot.capCibleRotation, getCalibratedHeading(robot));
  if (abs(error) < TOLERANCE_VIRAGE) {
      tourelle.write(SCAN_CENTER_ANGLE, NEUTRE_TOURELLE - robot.currentPitch); // Recentrer
      changeState(robot, MOVING_FORWARD);
  }
}

inline void handleManualTurningRightState(Robot& robot, int& pwmA, int& pwmB) {
  if (isObstacleDetected(robot)) {
    pwmA = 0;
    pwmB = 0;
    changeState(robot, IDLE); // Stop and go to IDLE if obstacle detected
  } else {
    int vitesseRotation = constrain(robot.vitesseCible, 0, VITESSE_ROTATION_MAX);
    pwmA = -vitesseRotation;
    pwmB = vitesseRotation;
  }
}

inline void handleFollowHeadingState(Robot& robot, int& pwmA, int& pwmB) {
  if (millis() - robot.lastCliffCheckTime > CLIFF_CHECK_INTERVAL_MS) {
      robot.stateBeforeGroundCheck = robot.currentState;
      changeState(robot, CHECKING_GROUND);
      return;
  }
  if (isObstacleDetected(robot)) {
      changeState(robot, OBSTACLE_AVOIDANCE); // Obstacle: scan for a new path
  } else {
    float errorFollow = calculateHeadingError(robot.Ncap, getCalibratedHeading(robot));
    int adjustment = Kp_HEADING * errorFollow;
    pwmA = robot.vitesseCible + adjustment;
    pwmB = robot.vitesseCible - adjustment;
  }
}

inline void handleMaintainHeadingState(Robot& robot, int& pwmA, int& pwmB) {
  float errorMaintain = calculateHeadingError(robot.Ncap, getCalibratedHeading(robot));
  int adjustment = Kp_HEADING * errorMaintain;
  pwmA = adjustment;
  pwmB = -adjustment;
}

inline void handleObstacleAvoidanceState(Robot& robot, int& pwmA, int& pwmB) {
  switch (robot.obstacleAvoidanceState) {
    case AVOID_START:
      Arret();
      // If the front switch is still pressed AND the obstacle was NOT detected by laser, immediately back up
      if (digitalRead(INTERUPTPIN) == LOW && !robot.obstacleDetectedByLaser) {
          robot.obstacleAvoidanceState = AVOID_BACKUP;
      } else {
          robot.obstacleAvoidanceState = AVOID_QUICK_SCAN_LEFT;
      }
      break;

    case AVOID_QUICK_SCAN_LEFT:
      tourelle.write(QUICK_SCAN_LEFT_ANGLE, NEUTRE_TOURELLE - robot.currentPitch); // Move turret to 70 degrees (left side)
      robot.turretMoveStartTime = millis();
      robot.obstacleAvoidanceState = AVOID_WAIT_FOR_LEFT_SCAN;
      break;

    case AVOID_WAIT_FOR_LEFT_SCAN:
      if (millis() - robot.turretMoveStartTime > TURRET_MOVE_TIME_MS) {
        if (vl53.dataReady()) {
          robot.distanceLaser = vl53.readRangeContinuousMillimeters() / MM_TO_CM_DIVISOR;
        }
        if (robot.distanceLaser < LASER_OBSTACLE_THRESHOLD_CM && robot.distanceLaser > 0) {
          if (DEBUG_MODE) Serial.println("OBSTACLE_AVOIDANCE: Side obstacle detected, backing up.");
          robot.obstacleAvoidanceState = AVOID_BACKUP;
        } else {
          robot.obstacleAvoidanceState = AVOID_QUICK_SCAN_RIGHT;
        }
      }
      break;

    case AVOID_QUICK_SCAN_RIGHT:
      tourelle.write(QUICK_SCAN_RIGHT_ANGLE, NEUTRE_TOURELLE - robot.currentPitch); // Move turret to 110 degrees (right side)
      robot.turretMoveStartTime = millis();
      robot.obstacleAvoidanceState = AVOID_WAIT_FOR_RIGHT_SCAN;
      break;

    case AVOID_WAIT_FOR_RIGHT_SCAN:
      if (millis() - robot.turretMoveStartTime > TURRET_MOVE_TIME_MS) {
        if (vl53.dataReady()) {
          robot.distanceLaser = vl53.readRangeContinuousMillimeters() / MM_TO_CM_DIVISOR;
        }
        if (robot.distanceLaser < LASER_OBSTACLE_THRESHOLD_CM && robot.distanceLaser > 0) {
          if (DEBUG_MODE) Serial.println("OBSTACLE_AVOIDANCE: Side obstacle detected, backing up.");
          robot.obstacleAvoidanceState = AVOID_BACKUP;
        } else {
          robot.obstacleAvoidanceState = AVOID_CENTER_TURRET;
        }
      }
      break;

    case AVOID_CENTER_TURRET:
      tourelle.write(SCAN_CENTER_ANGLE, NEUTRE_TOURELLE - robot.currentPitch); // Return turret to center
      robot.turretMoveStartTime = millis();
      robot.obstacleAvoidanceState = AVOID_WAIT_FOR_CENTER;
      break;

    case AVOID_WAIT_FOR_CENTER:
      if (millis() - robot.turretMoveStartTime > TURRET_MOVE_TIME_MS) {
        robot.obstacleAvoidanceState = AVOID_FULL_SCAN_START;
      }
      break;

    case AVOID_FULL_SCAN_START:
      robot.currentScanAngleH = SCAN_H_START_ANGLE;
      robot.bestAvoidAngle = SCAN_CENTER_ANGLE; // Default to center
      for (int i = 0; i < SCAN_DISTANCE_ARRAY_SIZE; i++) {
        robot.scanDistances[i] = 0; // Initialize scan distances
      }
      tourelle.write(robot.currentScanAngleH, NEUTRE_TOURELLE - robot.currentPitch); // Start scan
      robot.lastActionTime = millis();
      robot.obstacleAvoidanceState = AVOID_FULL_SCAN_STEP;
      if (DEBUG_MODE) Serial.println("OBSTACLE_AVOIDANCE: Starting full scan.");
      break;

    case AVOID_FULL_SCAN_STEP:
      if (millis() - robot.lastActionTime >= SCAN_DELAY_MS) {
        robot.lastActionTime = millis();

        // Read distance at current angle
        int currentDistance = robot.distanceLaser > 0 ? robot.distanceLaser : robot.dusm;
        if (currentDistance > 0) {
          robot.scanDistances[robot.currentScanAngleH] = currentDistance;
        }

        robot.currentScanAngleH += SCAN_H_STEP;
        if (robot.currentScanAngleH > SCAN_H_END_ANGLE) {
          robot.obstacleAvoidanceState = AVOID_FULL_SCAN_FINISH;
        } else {
          tourelle.write(robot.currentScanAngleH, NEUTRE_TOURELLE - robot.currentPitch); // Continue scanning
        }
      }
      break;

    case AVOID_FULL_SCAN_FINISH: { // Added curly braces to create a new scope
      // Scan complete, analyze results
      int maxDistance = 0;
      for (int i = SCAN_H_START_ANGLE; i <= SCAN_H_END_ANGLE; i += SCAN_H_STEP) {
        if (robot.scanDistances[i] > maxDistance) {
          maxDistance = robot.scanDistances[i];
          robot.bestAvoidAngle = i;
        }
      }

      if (maxDistance > LASER_OBSTACLE_THRESHOLD_CM) { // Found a clear path
        if (DEBUG_MODE) {
          Serial.print("OBSTACLE_AVOIDANCE: Clear path at ");
          Serial.print(robot.bestAvoidAngle);
          Serial.print(" deg with ");
          Serial.print(maxDistance);
          Serial.println(" cm.");
        }
        // Determine turn direction and set target heading
        float currentHeading = getCalibratedHeading(robot);
        float targetHeading = currentHeading + (robot.bestAvoidAngle - SCAN_CENTER_ANGLE); // Adjust current heading by relative scan angle
        
        // Normalize targetHeading
        if (targetHeading > 360) targetHeading -= 360;
        if (targetHeading < 0) targetHeading += 360;

        robot.capCibleRotation = targetHeading;
        
        if (robot.bestAvoidAngle < SCAN_CENTER_ANGLE) {
          changeState(robot, TURNING_LEFT);
        } else if (robot.bestAvoidAngle > SCAN_CENTER_ANGLE) {
          changeState(robot, TURNING_RIGHT);
        } else {
          // If best angle is center, just move forward
          changeState(robot, MOVING_FORWARD);
        }
        robot.vitesseCible = robot.speedSlow;
        tourelle.write(SCAN_CENTER_ANGLE, NEUTRE_TOURELLE - robot.currentPitch); // Center turret
      } else { // No clear path found, initiate backup maneuver
        if (DEBUG_MODE) Serial.println("OBSTACLE_AVOIDANCE: No clear path, backing up.");
        robot.obstacleAvoidanceState = AVOID_BACKUP;
      }
      break;
    } // End of added curly braces

    case AVOID_BACKUP:
      // Back up for a short duration, then re-scan.
      if (!robot.actionStarted) {
        robot.lastActionTime = millis();
        robot.actionStarted = true;
        if (DEBUG_MODE) Serial.println("AVOID_BACKUP: Starting backup.");
      }

      if (millis() - robot.lastActionTime < AVOID_BACKUP_DURATION_MS) { // Back up for 1 second
        pwmA = -robot.speedSlow;
        pwmB = -robot.speedSlow;
      } else {
        if (DEBUG_MODE) Serial.println("AVOID_BACKUP: Backup complete. Rescanning.");
        if (robot.consecutiveAvoidManeuvers >= MAX_CONSECUTIVE_AVOID_MANEUVERS) {
          if (DEBUG_MODE) Serial.println("AVOID_MANEUVER: Max consecutive avoid maneuvers reached. Giving up.");
          robot.stateBeforeHeadAnimation = IDLE;
          robot.currentHeadAnimation = ANIM_SHAKE_NO;
          robot.headAnimStartTime = millis();
          robot.headAnimCycles = 3;
          changeState(robot, ANIMATING_HEAD);
        } else {
          changeState(robot, OBSTACLE_AVOIDANCE); // Now try scanning again
        }
      }
      break;
  }
}

inline void handleScanning3DState(Robot& robot) {
  if (!robot.actionStarted) {
    // Initialize 3D scan
    robot.actionStarted = true;
    robot.currentScanAngleV = SCAN_V_START_ANGLE;
    robot.currentScanAngleH = SCAN_H_START_ANGLE;
    tourelle.write(robot.currentScanAngleH, robot.currentScanAngleV);
    robot.lastScanTime = millis();
    if (DEBUG_MODE) Serial.println(F("Starting 3D Scan."));
  }

  if (millis() - robot.lastScanTime >= SCAN_DELAY_MS) {
    robot.lastScanTime = millis();

    // 1. Read and report distance at current H,V
    int dist = robot.distanceLaser;
    Serial.print("3D:");
    Serial.print(robot.currentScanAngleH); Serial.print(",");
    Serial.print(robot.currentScanAngleV); Serial.print(",");
    Serial.println(dist);
    
    // 2. Increment horizontal angle
    robot.currentScanAngleH += SCAN_H_STEP;

    // 3. Check if horizontal scan row is finished
    if (robot.currentScanAngleH > SCAN_H_END_ANGLE) {
      // Row finished, move to next vertical angle
      robot.currentScanAngleV += SCAN_V_STEP;
      robot.currentScanAngleH = SCAN_H_START_ANGLE; // Reset horizontal angle

      // 4. Check if vertical scan is finished
      if (robot.currentScanAngleV > SCAN_V_END_ANGLE) {
        // 3D Scan is complete
        if (DEBUG_MODE) Serial.println(F("3D Scan Complete."));
        tourelle.write(SCAN_CENTER_ANGLE, NEUTRE_TOURELLE - robot.currentPitch); // Center turret
        changeState(robot, IDLE);
        return;
      }
    }
    
    // 5. Move to next position
    tourelle.write(robot.currentScanAngleH, robot.currentScanAngleV);
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
        tone(BUZZER_PIN, SENTRY_ALARM_TONE, SENTRY_TRACKING_DURATION_MS);
        robot.lastActionTime = millis();
        if (DEBUG_MODE) Serial.println("SENTRY: Tracking intruder.");
      }

      if (millis() - robot.lastActionTime > SENTRY_TRACKING_DURATION_MS) {
        if (DEBUG_MODE) Serial.println("SENTRY: Tracking finished.");
        PhareEteint();
        // noTone is not needed as tone() was given a duration
        tourelle.write(SCAN_CENTER_ANGLE, NEUTRE_TOURELLE - robot.currentPitch);
        robot.sentryState = SENTRY_IDLE;
        robot.actionStarted = false;
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

  // --- Universal E-Stop ---
  if (digitalRead(INTERUPTPIN) == LOW) {
      Arret();
      changeState(robot, OBSTACLE_AVOIDANCE); // Trigger obstacle avoidance instead of stopping
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
    case TURNING_LEFT:
      handleTurningLeftState(robot, pwmA, pwmB);
      break;
    case MANUAL_TURNING_LEFT:
      handleManualTurningLeftState(robot, pwmA, pwmB);
      break;
    case TURNING_RIGHT:
      handleTurningRightState(robot, pwmA, pwmB);
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
    case SCANNING:
      handleScanning(robot);
      break;
    case SCANNING_3D:
      handleScanning3DState(robot);
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

inline float calculateHeadingError(float target, float current) {
  float error = target - current;
  if (error > 180) error -= 360;
  else if (error < -180) error += 360;
  return error;
}

#endif // FONCTIONS_MOTRICES_H