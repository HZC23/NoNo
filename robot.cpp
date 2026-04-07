#include "robot.h"
#include "config.h"

void initializeRobot(Robot& robot) {
    robot.speedAvg = VITESSE_MOYENNE;
    robot.targetSpeed = 0; // Initialize at 0, will be set by modes
    robot.speedSlow = VITESSE_LENTE;
    robot.speedRotation = VITESSE_ROTATION;
    robot.speedRotationMax = VITESSE_ROTATION_MAX;
    robot.turnTolerance = TOLERANCE_VIRAGE;
    robot.KpHeading = Kp_HEADING;
    robot.motorBCalibration = CALIBRATION_MOTEUR_B;
    robot.minSpeedToMove = MIN_SPEED_TO_MOVE;
    robot.accelRate = ACCEL_RATE;
    robot.diffStrength = DIFF_STRENGTH;
    robot.fwdDiffCoeff = FWD_DIFF_COEFF;
    robot.servoNeutralDir = NEUTRE_DIRECTION;
    robot.servoNeutralTurret = NEUTRE_TOURELLE;
    robot.servoDirMin = SERVO_DIR_MIN;
    robot.servoDirMax = SERVO_DIR_MAX;
    robot.servoAngleHeadDown = ANGLE_TETE_BASSE;
    robot.servoAngleGround = ANGLE_SOL;
    robot.avoidBackupDuration = AVOID_BACKUP_DURATION_MS;
    robot.minDistForValidPath = MIN_DIST_FOR_VALID_PATH;
    robot.turretMoveTime = TURRET_MOVE_TIME_MS;
    robot.turretScanDelay = SCAN_DELAY_MS;
    robot.anglePenaltyFactor = ANGLE_PENALTY_FACTOR;
    robot.seuilVide = SEUIL_VIDE;
    robot.laserTimingBudget = VL53L1X_TIMING_BUDGET_US;
    robot.laserInterMeasurementPeriod = VL53L1X_INTER_MEASUREMENT_PERIOD_MS;
    robot.initialAutonomousDelay = INITIAL_AUTONOMOUS_DELAY_MS;
    robot.pivotAngleThreshold = SEUIL_BASCULE_DIRECTION;
    robot.compassInverted = COMPASS_IS_INVERTED;
    robot.currentState = IDLE;
    robot.obstacleAvoidanceState = AVOID_IDLE;
    robot.activeCommMode = DEFAULT_COMM_MODE;
}
