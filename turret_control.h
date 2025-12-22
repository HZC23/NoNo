#ifndef TURRET_CONTROL_H
#define TURRET_CONTROL_H

#include <Arduino.h>
#include "hardware.h"
#include "state.h"
#include "compass.h" // For pitch calculation
#include "config.h"  // For all constants

// Make hardware objects available from NoNo.ino
extern Tourelle tourelle;
extern VL53L1X *vl53;

// --- Turret Configuration ---
#define TURRET_UPDATE_INTERVAL 30 // milliseconds, for non-blocking updates
#define TURRET_TILT_STABILITY_FACTOR 0.8 // How much to correct tilt based on pitch

// Predictive Scan (while moving forward)
#define PREDICTIVE_SCAN_MIN_ANGLE 60
#define PREDICTIVE_SCAN_MAX_ANGLE 120
#define PREDICTIVE_SCAN_STEP 2

// Wide Search Scan (when stopped/searching)
#define WIDE_SCAN_MIN_ANGLE 10
#define WIDE_SCAN_MAX_ANGLE 170
#define WIDE_SCAN_STEP 5

// --- Turret State Variables ---
extern unsigned long lastTurretUpdate;
extern int turretPanAngle;
extern int turretTiltAngle;
extern int turretScanDirection;

// --- Turret Functions ---
void stabilizeTurretTilt(Robot& robot);
void updateTurret(Robot& robot, bool isMovingForward);
void syncTurretWithSteering(int steeringAngle);
int findClearestPath();

#endif // TURRET_CONTROL_H