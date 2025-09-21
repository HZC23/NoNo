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

// --- IMPLEMENTATIONS ---

inline void changeState(Robot& robot, RobotState newState) {
  if (robot.currentState == newState) return;
  
  robot.currentState = newState;
  robot.actionStarted = false;
  robot.lastActionTime = millis();
  
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
      changeState(robot, IDLE);
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
    case MANUAL_FORWARD:
      if (robot.dusm < DMARGE) { // Obstacle detected
          changeState(robot, IDLE);
      } else {
          pwmA = robot.vitesseCible;
          pwmB = robot.vitesseCible;
      }
      break;
      
    case MOVING_BACKWARD:
    case MANUAL_BACKWARD:
      pwmA = -robot.vitesseCible;
      pwmB = -robot.vitesseCible;
      break;
      
    case TURNING_LEFT:
    case MANUAL_TURNING_LEFT:
      pwmA = -VITESSE_ROTATION;
      pwmB = VITESSE_ROTATION;
      if (robot.currentState == TURNING_LEFT) {
          float error = calculateHeadingError(robot.capCibleRotation, getCalibratedHeading(robot));
          if (abs(error) < TOLERANCE_VIRAGE) changeState(robot, IDLE);
      }
      break;
       
    case TURNING_RIGHT:
    case MANUAL_TURNING_RIGHT:
      pwmA = VITESSE_ROTATION;
      pwmB = -VITESSE_ROTATION;
      if (robot.currentState == TURNING_RIGHT) {
          float error = calculateHeadingError(robot.capCibleRotation, getCalibratedHeading(robot));
          if (abs(error) < TOLERANCE_VIRAGE) changeState(robot, IDLE);
      }
      break;
      
    case FOLLOW_HEADING:
      if (robot.dusm < DMARGE) {
          changeState(robot, MAINTAIN_HEADING); // Stop if obstacle
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

    case SMART_AVOIDANCE:
      if (robot.dusm < DMARGE + 15) { // More cautious distance for this mode
        changeState(robot, SCANNING_ENVIRONMENT);
      } else {
        // Move forward, following the current Ncap
        robot.vitesseCible = VITESSE_MOYENNE;
        float errorFollow = calculateHeadingError(robot.Ncap, getCalibratedHeading(robot));
        int adjustment = Kp_HEADING * errorFollow;
        pwmA = robot.vitesseCible + adjustment;
        pwmB = robot.vitesseCible - adjustment;
      }
      break;

    case SCANNING_ENVIRONMENT:
      // For now, just stop and wait for a new command.
      // A real implementation would scan and find a new path.
      pwmA = 0;
      pwmB = 0;
      if (DEBUG_MODE) Serial.println("Obstacle detected in SMART_AVOIDANCE. Scanning... (placeholder)");
      // In a real scenario, you'd trigger a scan and then change state based on the result.
      changeState(robot, IDLE); 
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