#ifndef FONCTIONS_MOTRICES_H
#define FONCTIONS_MOTRICES_H

#include "robot.h"

// --- Main State Machine Functions ---
void updateMotorControl(Robot& robot);
void changeState(Robot& robot, RobotState newState);

#endif // FONCTIONS_MOTRICES_H
