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
    bool laserObstacle = (robot.distanceLaser < LASER_OBSTACLE_THRESHOLD_CM && robot.distanceLaser > 0);
    
    robot.obstacleDetectedByLaser = laserObstacle; // Set the flag if laser detects too close

    return ultraSonicObstacle || laserObstacle;
}

inline void changeState(Robot& robot, RobotState newState) {
  if (robot.currentState == newState) return;
  
  robot.currentState = newState;
  robot.actionStarted = false;
  robot.lastActionTime = millis();
  
  if (newState == OBSTACLE_AVOIDANCE) {
    robot.consecutiveAvoidManeuvers++;
    robot.obstacleAvoidanceState = AVOID_START; // Reset the sub-state
  } else {
    robot.consecutiveAvoidManeuvers = 0; // Reset counter on other state changes
  }

  if (newState != AVOID_MANEUVER) {
    robot.hasReculed = false;
    robot.hasTurned = false;
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

inline void updateMotorControl(Robot& robot) {
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
      pwmA = 0;
      pwmB = 0;
      break;

    case MOVING_FORWARD:
      if (robot.currentNavMode == AUTONOMOUS_CONTROL && isObstacleDetected(robot)) { // Obstacle detected in autonomous mode
          changeState(robot, OBSTACLE_AVOIDANCE);
      } else {
          pwmA = robot.vitesseCible;
          pwmB = robot.vitesseCible;
      }
      break;

    case MOVING_BACKWARD:
      pwmA = -robot.vitesseCible;
      pwmB = -robot.vitesseCible;
      break;

    case MANUAL_BACKWARD:
      if (isObstacleDetected(robot)) {
        pwmA = 0;
        pwmB = 0;
        changeState(robot, IDLE); // Stop and go to IDLE if obstacle detected
      } else {
        pwmA = -robot.vitesseCible;
        pwmB = -robot.vitesseCible;
      }
      break;

    case MANUAL_FORWARD:
      if (isObstacleDetected(robot)) {
        pwmA = 0;
        pwmB = 0;
        changeState(robot, IDLE); // Stop and go to IDLE if obstacle detected
      } else {
        pwmA = robot.vitesseCible;
        pwmB = robot.vitesseCible;
      }
      break;
      
    case TURNING_LEFT:
      if (!robot.actionStarted) {
        robot.lastActionTime = millis();
        robot.actionStarted = true;
      }
      
      // Timeout after 5 seconds of turning
      if (millis() - robot.lastActionTime > TURNING_TIMEOUT_MS) {
        if (DEBUG_MODE) Serial.println("TURNING_LEFT: Timeout");
        changeState(robot, MOVING_FORWARD); 
        break;
      }

      pwmA = VITESSE_ROTATION;
      pwmB = -VITESSE_ROTATION;
      
      {
          float error = calculateHeadingError(robot.capCibleRotation, getCalibratedHeading(robot));
          if (abs(error) < TOLERANCE_VIRAGE) {
              changeState(robot, MOVING_FORWARD);
          }
      }
      break;

    case MANUAL_TURNING_LEFT:
      if (isObstacleDetected(robot)) {
        pwmA = 0;
        pwmB = 0;
        changeState(robot, IDLE); // Stop and go to IDLE if obstacle detected
      } else {
        pwmA = VITESSE_ROTATION;
        pwmB = -VITESSE_ROTATION;
      }
      break;
       
    case TURNING_RIGHT:
      if (!robot.actionStarted) {
        robot.lastActionTime = millis();
        robot.actionStarted = true;
      }
      
      // Timeout after 5 seconds of turning
      if (millis() - robot.lastActionTime > TURNING_TIMEOUT_MS) {
        if (DEBUG_MODE) Serial.println("TURNING_RIGHT: Timeout");
        changeState(robot, MOVING_FORWARD); 
        break;
      }

      pwmA = -VITESSE_ROTATION;
      pwmB = VITESSE_ROTATION;
      
      {
          float error = calculateHeadingError(robot.capCibleRotation, getCalibratedHeading(robot));
          if (abs(error) < TOLERANCE_VIRAGE) {
              changeState(robot, MOVING_FORWARD);
          }
      }
      break;

    case MANUAL_TURNING_RIGHT:
      if (isObstacleDetected(robot)) {
        pwmA = 0;
        pwmB = 0;
        changeState(robot, IDLE); // Stop and go to IDLE if obstacle detected
      } else {
        pwmA = -VITESSE_ROTATION;
        pwmB = VITESSE_ROTATION;
      }
      break;
      
    case FOLLOW_HEADING:
      if (isObstacleDetected(robot)) {
          changeState(robot, OBSTACLE_AVOIDANCE); // Obstacle: scan for a new path
      } else {
        float errorFollow = calculateHeadingError(robot.Ncap, getCalibratedHeading(robot));
        int adjustment = Kp_HEADING * errorFollow;
        pwmA = robot.vitesseCible + adjustment;
        pwmB = robot.vitesseCible - adjustment;
      }
      break;
        
    case MAINTAIN_HEADING:
      {
        float errorMaintain = calculateHeadingError(robot.Ncap, getCalibratedHeading(robot));
        int adjustment = Kp_HEADING * errorMaintain;
        pwmA = adjustment;
        pwmB = -adjustment;
      }
      break;

    case OBSTACLE_AVOIDANCE:
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
          tourelle.write(QUICK_SCAN_LEFT_ANGLE, NEUTRE_TOURELLE); // Move turret to 70 degrees (left side)
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
          tourelle.write(QUICK_SCAN_RIGHT_ANGLE, NEUTRE_TOURELLE); // Move turret to 110 degrees (right side)
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
          tourelle.write(SCAN_CENTER_ANGLE, NEUTRE_TOURELLE); // Return turret to center
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
          tourelle.write(robot.currentScanAngleH, NEUTRE_TOURELLE); // Start scan
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
              tourelle.write(robot.currentScanAngleH, NEUTRE_TOURELLE); // Continue scanning
            }
          }
          break;

        case AVOID_FULL_SCAN_FINISH:
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
            robot.vitesseCible = VITESSE_LENTE;
            tourelle.write(SCAN_CENTER_ANGLE, NEUTRE_TOURELLE); // Center turret
          } else { // No clear path found, initiate backup maneuver
            if (DEBUG_MODE) Serial.println("OBSTACLE_AVOIDANCE: No clear path, backing up.");
            robot.obstacleAvoidanceState = AVOID_BACKUP;
          }
          break;

        case AVOID_BACKUP:
          // Back up for a short duration, then re-scan.
          if (!robot.actionStarted) {
            robot.lastActionTime = millis();
            robot.actionStarted = true;
            if (DEBUG_MODE) Serial.println("AVOID_MANEUVER: Starting backup.");
          }

          if (millis() - robot.lastActionTime < AVOID_BACKUP_DURATION_MS) { // Back up for 1 second
            pwmA = -VITESSE_LENTE;
            pwmB = -VITESSE_LENTE;
          } else {
            if (DEBUG_MODE) Serial.println("AVOID_MANEUVER: Backup complete. Rescanning.");
            if (robot.consecutiveAvoidManeuvers >= MAX_CONSECUTIVE_AVOID_MANEUVERS) {
              if (DEBUG_MODE) Serial.println("AVOID_MANEUVER: Max consecutive avoid maneuvers reached. Going IDLE.");
              changeState(robot, IDLE); // Give up and go IDLE
            } else {
              changeState(robot, OBSTACLE_AVOIDANCE); // Now try scanning again
            }
          }
          break;
      }
      break;

    case SMART_AVOIDANCE:
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
      break;

    case SENTRY_MODE:
      pwmA = 0;
      pwmB = 0;
      if (digitalRead(PIR) == HIGH) {
        changeState(robot, SENTRY_ALARM);
      }
      break;

    case SENTRY_ALARM:
      pwmA = 0;
      pwmB = 0;
      // Flash headlight for 5 seconds
      if (millis() - robot.lastActionTime > SENTRY_ALARM_DURATION_MS) {
        PhareEteint();
        changeState(robot, SENTRY_MODE);
      } else {
        // Flash every 250ms
        if ((millis() / SENTRY_FLASH_INTERVAL_MS) % SENTRY_ALARM_BLINK_DIVISOR == 0) {
          PhareAllume();
        } else {
          PhareEteint();
        }
      }
      break;
    
default:
      pwmA = 0;
      pwmB = 0;
      break;
  }
  
  pwmB = (int)(pwmB * CALIBRATION_MOTEUR_B);
  
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