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
  motorA.motorBrake(255);
  motorB.motorBrake(255);
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
      if (millis() - robot.lastActionTime > 5000) {
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
      if (millis() - robot.lastActionTime > 5000) {
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
      // If the front switch is still pressed AND the obstacle was NOT detected by laser, immediately back up
      if (digitalRead(INTERUPTPIN) == LOW && !robot.obstacleDetectedByLaser) {
          changeState(robot, AVOID_MANEUVER);
          return;
      }

      if (!robot.actionStarted) {
        // Initial setup for obstacle avoidance
        Arret(); // Stop the robot

        // Quick side check with laser
        tourelle.write(70, 90); // Move turret to 70 degrees (left side)
        delay(200); // Give turret time to move
        if (vl53.dataReady()) {
          robot.distanceLaser = vl53.readRangeContinuousMillimeters() / 10;
        }
        if (robot.distanceLaser < LASER_OBSTACLE_THRESHOLD_CM && robot.distanceLaser > 0) {
          if (DEBUG_MODE) Serial.println("OBSTACLE_AVOIDANCE: Side obstacle detected, backing up.");
          changeState(robot, AVOID_MANEUVER);
          return;
        }
        tourelle.write(110, 90); // Move turret to 110 degrees (right side)
        delay(200); // Give turret time to move
        if (vl53.dataReady()) {
          robot.distanceLaser = vl53.readRangeContinuousMillimeters() / 10;
        }
        if (robot.distanceLaser < LASER_OBSTACLE_THRESHOLD_CM && robot.distanceLaser > 0) {
          if (DEBUG_MODE) Serial.println("OBSTACLE_AVOIDANCE: Side obstacle detected, backing up.");
          changeState(robot, AVOID_MANEUVER);
          return;
        }
        tourelle.write(SCAN_CENTER_ANGLE, 90); // Return turret to center
        delay(200); // Give turret time to move

        robot.currentScanAngleH = SCAN_H_START_ANGLE;
        robot.bestAvoidAngle = SCAN_CENTER_ANGLE; // Default to center
        for (int i = 0; i <= 180; i++) {
          robot.scanDistances[i] = 0; // Initialize scan distances
        }
        tourelle.write(robot.currentScanAngleH, 90); // Start scan
        robot.lastActionTime = millis();
        robot.actionStarted = true;
        if (DEBUG_MODE) Serial.println("OBSTACLE_AVOIDANCE: Starting full scan.");
      }

      if (millis() - robot.lastActionTime >= SCAN_DELAY_MS) {
        robot.lastActionTime = millis();

        // Read distance at current angle
        int currentDistance = robot.distanceLaser > 0 ? robot.distanceLaser : robot.dusm;
        if (currentDistance > 0) {
          robot.scanDistances[robot.currentScanAngleH] = currentDistance;
        }

        robot.currentScanAngleH += SCAN_H_STEP;
        if (robot.currentScanAngleH > SCAN_H_END_ANGLE) {
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
            tourelle.write(SCAN_CENTER_ANGLE, 90); // Center turret
          } else { // No clear path found, initiate backup maneuver
            if (DEBUG_MODE) Serial.println("OBSTACLE_AVOIDANCE: No clear path, backing up.");
            changeState(robot, AVOID_MANEUVER);
          }
        } else {
          tourelle.write(robot.currentScanAngleH, 90); // Continue scanning
        }
      }
      break;

    case AVOID_MANEUVER:
      // Back up for a short duration, then re-scan.
      if (!robot.actionStarted) {
        robot.lastActionTime = millis();
        robot.actionStarted = true;
        if (DEBUG_MODE) Serial.println("AVOID_MANEUVER: Starting backup.");
      }

      if (millis() - robot.lastActionTime < 1000) { // Back up for 1 second
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
      if (millis() - robot.lastActionTime > 5000) {
        PhareEteint();
        changeState(robot, SENTRY_MODE);
      } else {
        // Flash every 250ms
        if ((millis() / 250) % 2 == 0) {
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
  
  motorA.motorGo(constrain(pwmA, -255, 255));
  motorB.motorGo(constrain(pwmB, -255, 255));
}

inline float calculateHeadingError(float target, float current) {
  float error = target - current;
  if (error > 180) error -= 360;
  else if (error < -180) error += 360;
  return error;
}

#endif // FONCTIONS_MOTRICES_H