#ifndef FONCTIONS_MOTRICES_H
#define FONCTIONS_MOTRICES_H

#include "robot.h"

// --- Main State Machine Functions ---
void updateMotorControl(Robot& robot);
void changeState(Robot& robot, RobotState newState, ObstacleAvoidanceState avoidState = AVOID_IDLE);
void calculateManualPwm(Robot& robot, int& outPwmA, int& outPwmB); // New prototype

#endif // FONCTIONS_MOTRICES_H
