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
inline int findClearestPath(Robot& robot) {
    int bestAngle = -1;
    float maxScore = -9999;
    
    // Balayage
    // On suppose que scanDistances[] est rempli par une fonction de scan préalable
    // ou on effectue le scan ici.
    
    for (int angle = SCAN_H_START_ANGLE; angle <= SCAN_H_END_ANGLE; angle += SCAN_H_STEP) {
        // Positionner la tourelle et lire (version simplifiée, idéalement non bloquante)
        tourelle.write(angle, robot.servoNeutralTurret);
        delay(robot.turretScanDelay); 
        
        // Lecture combinée Laser + Ultrason pour la robustesse
        int distLaser = vl53->readRangeContinuousMillimeters() / 10;
        int distUS = robot.dusm; // Supposons que c'est mis à jour via sensor_update_task
        
        // On prend la distance la plus courte des deux pour la sécurité
        int effectiveDist = (distUS > 0 && distUS < distLaser) ? distUS : distLaser;
        if (effectiveDist <= 0) effectiveDist = robot.maxUltrasonicDistance; // Erreur ou infini

        // --- ALGORITHME DE PONDÉRATION ---
        // On favorise l'angle central (90°). 
        // Plus on s'éloigne de 90°, plus on "coûte" de points.
        float angleDeviation = abs(90 - angle);
        float score = effectiveDist - (angleDeviation * robot.anglePenaltyFactor);

        if (effectiveDist > robot.minDistForValidPath && score > maxScore) {
            maxScore = score;
            bestAngle = angle;
        }
    }

    return bestAngle;
}

#endif // TURRET_CONTROL_H