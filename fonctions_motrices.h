#ifndef FONCTIONS_MOTRICES_H
#define FONCTIONS_MOTRICES_H

#include "hardware.h"
#include "config.h"

// Forward declaration
struct Robot;

// --- Public Function Prototypes ---
const char* stateToString(RobotState state);
void changeState(Robot& robot, RobotState newState, ObstacleAvoidanceState avoidState = AVOID_IDLE);
void updateMotorControl(Robot& robot);

#endif // FONCTIONS_MOTRICES_H

