
#ifndef FONCTIONS_MOTRICES_H
#define FONCTIONS_MOTRICES_H

#include "config.h"
#include "state.h"
#include "compass.h" // For getCalibratedHeading

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
  if (robot.dusm < DARRET || digitalRead(INTERUPTPIN) == LOW) {
    Arret();
    if (robot.currentState == OBSTACLE_AVOIDANCE) {
      changeState(robot, AVOID_MANEUVER);
    } else if (robot.currentState == MOVING_FORWARD) {
      changeState(robot, IDLE);
    }
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
      pwmA = robot.vitesseCible;
      pwmB = robot.vitesseCible;
      // Servodirection.write(NEUTRE_DIRECTION); // Ackermann steering
      break;
      
    case MOVING_BACKWARD:
    case MANUAL_BACKWARD:
      pwmA = -robot.vitesseCible;
      pwmB = -robot.vitesseCible;
      // Servodirection.write(NEUTRE_DIRECTION); // Ackermann steering
      break;
      
    case TURNING_LEFT:
    case MANUAL_TURNING_LEFT:
      {
        pwmA = -VITESSE_ROTATION;
        pwmB = VITESSE_ROTATION;
        if (robot.currentState == TURNING_LEFT) {
            float error = calculateHeadingError(robot.capCibleRotation, getCalibratedHeading(robot));
            if (abs(error) < TOLERANCE_VIRAGE) changeState(robot, IDLE);
        }
      }
      break;
       
    case TURNING_RIGHT:
    case MANUAL_TURNING_RIGHT:
      {
        pwmA = VITESSE_ROTATION;
        pwmB = -VITESSE_ROTATION;
        if (robot.currentState == TURNING_RIGHT) {
            float error = calculateHeadingError(robot.capCibleRotation, getCalibratedHeading(robot));
            if (abs(error) < TOLERANCE_VIRAGE) changeState(robot, IDLE);
        }
      }
      break;
      
    case FOLLOW_HEADING:
      {
        float errorFollow = calculateHeadingError(robot.Ncap, getCalibratedHeading(robot));
        int adjustment = Kp_HEADING * errorFollow;
        pwmA = robot.vitesseCible + adjustment;
        pwmB = robot.vitesseCible - adjustment;
        // int servoAdjustment = constrain(errorFollow * SERVO_ADJUSTMENT_FACTOR, -SERVO_MAX_ADJUSTMENT, SERVO_MAX_ADJUSTMENT); // Ackermann steering
        // Servodirection.write(NEUTRE_DIRECTION + servoAdjustment); // Ackermann steering
      }
      break;
        
    case MAINTAIN_HEADING:
      {
        float errorMaintain = calculateHeadingError(robot.Ncap, getCalibratedHeading(robot));
        int adjustment = Kp_HEADING * errorMaintain;
        pwmA = adjustment;
        pwmB = -adjustment;
        // int servoAdjustmentMaintain = constrain(errorMaintain * SERVO_ADJUSTMENT_FACTOR, -SERVO_MAX_ADJUSTMENT, SERVO_MAX_ADJUSTMENT); // Ackermann steering
        // Servodirection.write(NEUTRE_DIRECTION + servoAdjustmentMaintain); // Ackermann steering
      }
      break;
    
    // Other states...
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
