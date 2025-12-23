#ifndef FONCTIONS_MOTRICES_H
#define FONCTIONS_MOTRICES_H

#include "config.h"
#include "state.h"
#include "compass.h" // For getCalibratedHeading
#include "support.h" // For Phare control
#include "turret_control.h"

// --- CONSTANTES PHYSIQUES ---
// Ajustez ces valeurs pour changer le "feeling" de la conduite
// These values are now configurable via SD card in config.txt
// and stored in the robot struct.

// --- PROTOTYPES ---
void changeState(Robot& robot, RobotState newState);
void Arret();
void updateMotorControl(Robot& robot);
float calculateHeadingError(float target, float current);
bool isObstacleDetected(Robot& robot);
inline int applySpeedRamp(float target, float current); // New prototype
inline int getDynamicMaxSteering(int currentSpeed); // New prototype
inline void calculateDifferentialDrive(int targetSpeed, int steeringAngleRelative, int& outPwmA, int& outPwmB); // New prototype
bool detectImpactOrStall(Robot& robot); // Renamed from isPhysicallyStuck
void handleEmergencyEvasionState(Robot& robot, int& pwmA, int& pwmB);
void handleStuckState(Robot& robot, int& pwmA, int& pwmB);

// --- IMPLEMENTATIONS ---

// 1. FONCTION DE LISSAGE (RAMPING)
// Lisse les accélérations et décélérations pour un mouvement plus réaliste et éviter les à-coups.
// Simule l'inertie du robot.
inline int applySpeedRamp(Robot& robot, float target, float current) {
    if (abs(target - current) < 1.0) return (int)target;
    // Formule de filtre passe-bas : Nouvelle = Ancienne + (Cible - Ancienne) * Facteur
    return (int)(current + (target - current) * robot.accelRate);
}

// 2. FONCTION DIRECTION DYNAMIQUE
// Adapte l'angle de braquage maximal en fonction de la vitesse actuelle du robot.
// À vitesse élevée, le braquage est réduit pour améliorer la stabilité et éviter le renversement.
inline int getDynamicMaxSteering(Robot& robot, int currentSpeed) {
    // Si vitesse faible (<100), braquage max autorisé.
    // Si vitesse max (255), on réduit le braquage de 40% environ.
    if (currentSpeed < 100) return robot.pivotAngleThreshold;
    
    float ratio = 1.0 - ((float)(currentSpeed - 100) / (255.0 - 100.0) * 0.4); 
    return (int)(robot.pivotAngleThreshold * ratio);
}

// 3. DIFFERENTIEL ELECTRONIQUE
// Calcule les vitesses PWM individuelles pour les moteurs gauche (A) et droit (B)
// en fonction de la vitesse cible globale et de l'angle de direction demandé.
// Ceci permet un mouvement de type "voiture" en virage, où la roue intérieure ralentit et l'extérieure accélère.
// Intègre des ajustements spécifiques pour les robots 4WD afin d'améliorer la maniabilité en virage.
inline void calculateDifferentialDrive(Robot& robot, int targetSpeed, int steeringAngleRelative, int& outPwmA, int& outPwmB) {
    // steeringAngleRelative : -robot.pivotAngleThreshold (Gauche) à +robot.pivotAngleThreshold (Droite)
    // targetSpeed : Vitesse de base souhaitée

    if (targetSpeed == 0) {
        outPwmA = 0;
        outPwmB = 0;
        return;
    }

    // Facteur de virage : de -1.0 (max gauche) à 1.0 (max droite)
    float turnFactor = (float)steeringAngleRelative / (float)robot.pivotAngleThreshold; 
    turnFactor = constrain(turnFactor, -1.0, 1.0); // Ensure it stays within bounds

    // En 4WD, la résistance au pivot est plus forte. 
    // On augmente le coefficient de différentiel pour aider le train arrière à pousser.
    float diff_coeff = robot.fwdDiffCoeff; // Use the new 4WD coefficient

    if (turnFactor > 0) { // Virage DROITE -> Roue Droite (B) ralentit
        outPwmA = targetSpeed; // Gauche (Extérieur) garde la vitesse ou accélère
        outPwmB = targetSpeed * (1.0 - (turnFactor * diff_coeff)); 
    } else { // Virage GAUCHE -> Roue Gauche (A) ralentit
        outPwmA = targetSpeed * (1.0 - (abs(turnFactor) * diff_coeff));
        outPwmB = targetSpeed; // Droite (Extérieur)
    }

    // Sécurité : ne pas laisser une roue s'arrêter totalement ou s'inverser en roulant
    // sauf si on est en mode Pivot (Tank Turn), which is handled by changeState
    if (targetSpeed != 0 && abs(turnFactor) < 0.8) { // Only apply if not trying to pivot and moving
        outPwmA = max(outPwmA, robot.minSpeedToMove);
        outPwmB = max(outPwmB, robot.minSpeedToMove);
    }
}

inline bool isObstacleDetected(Robot& robot) {
    bool ultraSonicObstacle = (robot.dusm < ULTRASONIC_OBSTACLE_THRESHOLD_CM && robot.dusm > 0);
    bool laserObstacle = (robot.laserInitialized && robot.distanceLaser < LASER_OBSTACLE_THRESHOLD_CM && robot.distanceLaser > 0);
    robot.obstacleDetectedByLaser = laserObstacle;
    return ultraSonicObstacle || laserObstacle;
}

// Function to manage the steering servo (Ackermann) and return the angle
inline int setAckermannAngle(Robot& robot, int angleError, int speed) {
    // Récupérer la limite dynamique
    int limit = getDynamicMaxSteering(robot, speed);
    
    // Clamper l'erreur à cette limite
    int clampedError = constrain(angleError, -limit, limit);

    int servoAngle = map(clampedError, -robot.pivotAngleThreshold, robot.pivotAngleThreshold, robot.servoDirMin, robot.servoDirMax);
    servoAngle = constrain(servoAngle, robot.servoDirMin, robot.servoDirMax);
    Servodirection.write(servoAngle);
    return servoAngle; // Retourne l'angle réel du servo
}




inline void changeState(Robot& robot, RobotState newState) {
  if (robot.currentState == newState) return;
  
  // Store the current state if we are transitioning to an animation state
  if (newState == ANIMATING_HEAD) {
    robot.stateBeforeHeadAnimation = robot.currentState;
  }
  
  robot.currentState = newState;
  robot.actionStarted = false;
  robot.lastActionTime = millis();
  
  // If returning to IDLE, reset curious look targets to re-center the head.
  if (newState == IDLE) {
    robot.curiousTargetH = robot.servoNeutralTurret;
    robot.curiousTargetV = robot.servoNeutralTurret;
  }

  if (newState == OBSTACLE_AVOIDANCE) {
    robot.consecutiveAvoidManeuvers++;
    robot.obstacleAvoidanceState = AVOID_START; // Reset the sub-state
  } else if (robot.currentState != OBSTACLE_AVOIDANCE) { // Avoid resetting if we are already in a sub-maneuver
    robot.consecutiveAvoidManeuvers = 0; // Reset counter on other state changes
  }

  // Reset sub-state machines when leaving their primary state
  if (newState != CHECKING_GROUND) {
    robot.groundCheckState = GC_START;
  }
  if (newState != ANIMATING_HEAD) {
    robot.currentHeadAnimation = ANIM_NONE;
  }
    if (newState == STUCK) {
      robot.isStuckConfirmed = true;
    } else if (robot.currentState == STUCK && newState != STUCK) {
      robot.isStuckConfirmed = false;
    }
    if (newState != EMERGENCY_EVASION) { robot.evasionState = EVADE_START; }


  if (DEBUG_MODE) { 
    // Serial.print("State -> "); 
    // Serial.println(stateToString(newState)); // stateToString not defined in this context
  }
}

inline void Arret() {
    // Freinage immédiat (bypass le ramping pour la sécurité)
    motorA.motorBrake();
    motorB.motorBrake();
}


// Helper function for smooth sine-wave based easing
inline float easeOutInSine(float t) {
    return 0.5 * (1 + sin(PI * (t - 0.5)));
}

inline void handleHeadAnimation(Robot& robot, bool changeStateOnFinish) {
  if (robot.currentHeadAnimation == ANIM_NONE) return;

  unsigned long elapsed = millis() - robot.headAnimStartTime;

  if (robot.currentHeadAnimation == ANIM_SHAKE_NO) {
    int cycleTime = HEAD_SHAKE_NO_CYCLE_DURATION_MS * 2; // Full back-and-forth cycle
    int totalDuration = robot.headAnimCycles * cycleTime;

    if (elapsed < totalDuration) {
      float progress = (float)(elapsed % cycleTime) / cycleTime;
      float eased_progress = easeOutInSine(progress);
      
      int angle = map(eased_progress, 0, 1, HEAD_SHAKE_NO_ANGLE_EXTREME_RIGHT, HEAD_SHAKE_NO_ANGLE_EXTREME_LEFT);
      tourelle.write(angle, robot.servoNeutralTurret);
    } else {
      if (changeStateOnFinish) {
        tourelle.write(robot.servoNeutralTurret, robot.servoNeutralTurret);
        changeState(robot, robot.stateBeforeHeadAnimation);
      } else {
        robot.currentHeadAnimation = ANIM_NONE; // Animation is over
      }
    }
  } else if (robot.currentHeadAnimation == ANIM_NOD_YES) {
    int cycleTime = HEAD_NOD_YES_CYCLE_DURATION_MS * 2; // Full up-and-down cycle
    int totalDuration = robot.headAnimCycles * cycleTime;

    if (elapsed < totalDuration) {
      float progress = (float)(elapsed % cycleTime) / cycleTime;
      float eased_progress = easeOutInSine(progress);

      int angle = map(eased_progress, 0, 1, HEAD_NOD_YES_ANGLE_DOWN, HEAD_NOD_YES_ANGLE_UP);
      tourelle.write(tourelle.getAngleHorizontal(), angle);
    } else {
      if (changeStateOnFinish) {
        tourelle.write(tourelle.getAngleHorizontal(), robot.servoNeutralTurret);
        changeState(robot, robot.stateBeforeHeadAnimation);
      } else {
        robot.currentHeadAnimation = ANIM_NONE; // Animation is over
      }
    }
  }
}

// --- STATE HANDLERS ---

inline void handleIdleState(Robot& robot, int& pwmA, int& pwmB) {
  pwmA = 0;
  pwmB = 0;
  // Let the main loop's updateTurret() handle idle movement
}

inline void handleMovingForwardState(Robot& robot, int& targetA, int& targetB) {
  if (detectImpactOrStall(robot)) { changeState(robot, STUCK); return; }
  if (millis() - robot.lastActionTime > STALL_DETECTION_TIMEOUT_MS) { changeState(robot, STUCK); return; }
  if (millis() - robot.lastCliffCheckTime > CLIFF_CHECK_INTERVAL_MS) {
      robot.stateBeforeGroundCheck = robot.currentState;
      changeState(robot, CHECKING_GROUND);
      return;
  }
  if (robot.currentNavMode == AUTONOMOUS_CONTROL && isObstacleDetected(robot)) {
      changeState(robot, OBSTACLE_AVOIDANCE);
  } else {
      // Marche avant droite
      Servodirection.write(robot.servoNeutralDir);
      tourelle.write(robot.servoNeutralTurret, robot.servoNeutralTurret);
      targetA = robot.vitesseCible;
      targetB = robot.vitesseCible;
  }
}

inline void handleMovingBackwardState(Robot& robot, int& pwmA, int& pwmB) {
  pwmA = -robot.vitesseCible;
  pwmB = -robot.vitesseCible;
}

inline void handleManualBackwardState(Robot& robot, int& pwmA, int& pwmB) {
  if (isObstacleDetected(robot)) {
    pwmA = 0;
    pwmB = 0;
    changeState(robot, IDLE);
  } else {
    pwmA = -robot.vitesseCible;
    pwmB = -robot.vitesseCible;
  }
}

inline void handleManualForwardState(Robot& robot, int& pwmA, int& pwmB) {
  if (isObstacleDetected(robot)) {
    pwmA = 0;
    pwmB = 0;
    changeState(robot, IDLE);
  } else {
    pwmA = robot.vitesseCible;
    pwmB = robot.vitesseCible;
  }
}

inline void handleManualTurningLeftState(Robot& robot, int& pwmA, int& pwmB) {
  if (isObstacleDetected(robot)) {
    pwmA = 0;
    pwmB = 0;
    changeState(robot, IDLE);
  } else {
    int vitesseRotation = constrain(robot.vitesseCible, 0, robot.speedRotationMax);
    pwmA = vitesseRotation;
    pwmB = -vitesseRotation;
  }
}

inline void handleManualTurningRightState(Robot& robot, int& pwmA, int& pwmB) {
  if (isObstacleDetected(robot)) {
    pwmA = 0;
    pwmB = 0;
    changeState(robot, IDLE);
  } else {
    int vitesseRotation = constrain(robot.vitesseCible, 0, robot.speedRotationMax);
    pwmA = -vitesseRotation;
    pwmB = vitesseRotation;
  }
}

// Gère le comportement du robot lorsqu'il doit suivre un cap spécifique (direction).
// Il ajuste dynamiquement la direction du servo et le différentiel des moteurs
// pour atteindre et maintenir le cap cible, avec gestion des obstacles et du blocage.
inline void handleFollowHeadingState(Robot& robot, int& targetA, int& targetB) {
  if (detectImpactOrStall(robot)) { changeState(robot, STUCK); return; }
  if (isObstacleDetected(robot)) { changeState(robot, OBSTACLE_AVOIDANCE); return; }

  float currentHeading = getCalibratedHeading(robot);
  float error = calculateHeadingError(robot.capCibleRotation, currentHeading);

  if (abs(error) < robot.turnTolerance) {
      changeState(robot, MOVING_FORWARD); 
      return;
  }

  // Si l'erreur est énorme (>30°), on pivote sur place (tank turn)
  // Le différentiel ne suffit plus.
  if (abs(error) > robot.pivotAngleThreshold) {
      Servodirection.write(robot.servoNeutralDir); 
      int pivotSpeed = robot.speedRotation;
      if (error > 0) { targetA = -pivotSpeed; targetB = pivotSpeed; } // Droite
      else { targetA = pivotSpeed; targetB = -pivotSpeed; } // Gauche
  } else {
      // Virage fluide (Ackermann + Différentiel)
      setAckermannAngle(robot, (int)error, robot.vitesseCible);
      syncTurretWithSteering(Servodirection.read());
      calculateDifferentialDrive(robot, robot.vitesseCible, (int)error, targetA, targetB);
      robot.currentSteeringBias = (int)error;
  }
}

inline void handleMaintainHeadingState(Robot& robot, int& pwmA, int& pwmB) {
  float errorMaintain = calculateHeadingError(robot.Ncap, getCalibratedHeading(robot));
  int adjustment = robot.KpHeading * errorMaintain;
  pwmA = adjustment;
  pwmB = -adjustment;
}

// *** REFACTORED OBSTACLE AVOIDANCE ***
// Gère le comportement d'évitement d'obstacles lorsque le robot est confronté à un obstacle direct.
// Ce mode implique un arrêt, un balayage pour trouver le chemin le plus clair,
// et une stratégie d'évasion si le robot est complètement bloqué (trap escape).
inline void handleObstacleAvoidanceState(Robot& robot, int& targetA, int& targetB) {
  // (Conserver la logique de "Trap Escape" et "Scanning" vue précédemment)
  // Pour la marche arrière, pas de différentiel complexe nécessaire
  switch (robot.obstacleAvoidanceState) {
    case AVOID_START: Arret(); robot.obstacleAvoidanceState = AVOID_FULL_SCAN_START; break;
    case AVOID_FULL_SCAN_START: {
      int bestAngle = findClearestPath(robot);
      if (bestAngle != -1) {
        float angleOffset = bestAngle - 90.0; 
        robot.capCibleRotation = getCalibratedHeading(robot) + angleOffset;
        if (robot.capCibleRotation < 0) robot.capCibleRotation += 360;
        if (robot.capCibleRotation >= 360) robot.capCibleRotation -= 360;
        changeState(robot, FOLLOW_HEADING);
      } else { robot.obstacleAvoidanceState = AVOID_BACKUP; }
      break;
    }
    case AVOID_BACKUP:
      if (!robot.actionStarted) { robot.lastActionTime = millis(); robot.actionStarted = true; tourelle.write(robot.servoNeutralTurret, robot.servoAngleHeadDown); }
      if (millis() - robot.lastActionTime < robot.avoidBackupDuration) { targetA = -robot.speedSlow; targetB = -robot.speedSlow; } 
      else { 
          if (robot.consecutiveAvoidManeuvers >= 3) {
            robot.consecutiveAvoidManeuvers = 0; 
            float randomTurn = random(120, 240);
            robot.capCibleRotation = getCalibratedHeading(robot) + randomTurn;
            if(robot.capCibleRotation >= 360) robot.capCibleRotation -= 360;
            changeState(robot, FOLLOW_HEADING);
          } else { changeState(robot, OBSTACLE_AVOIDANCE); }
      }
      break;
    default: robot.obstacleAvoidanceState = AVOID_START; break;
  }
}


// Gère la navigation intelligente en évitant les obstacles de manière fluide.
// Le robot scanne continuellement les alentours avec la tourelle et ajuste sa trajectoire
// en temps réel pour dévier des obstacles sans s'arrêter brusquement.
inline void handleSmartAvoidanceState(Robot& robot, int& targetA, int& targetB) {
  if (millis() - robot.lastCliffCheckTime > CLIFF_CHECK_INTERVAL_MS) {
      robot.stateBeforeGroundCheck = robot.currentState;
      changeState(robot, CHECKING_GROUND);
      return;
  }
  if (isObstacleDetected(robot)) { changeState(robot, OBSTACLE_AVOIDANCE); return; }

  // 1. Logique de regard (Scanning)
  static int lookDir = 0; 
  static unsigned long lastLookTime = 0;
  if (millis() - lastLookTime > 200) {
      lastLookTime = millis();
      switch(lookDir) {
          case 0: tourelle.write(robot.servoNeutralTurret, robot.servoNeutralTurret); lookDir = 1; break;
          case 1: tourelle.write(robot.servoNeutralTurret + 30, robot.servoNeutralTurret); lookDir = 2; break; 
          case 2: tourelle.write(robot.servoNeutralTurret - 30, robot.servoNeutralTurret); lookDir = 0; break; 
      }
  }

  // 2. Calcul de la déviation
  int dist = robot.distanceLaser;
  int steeringBias = 0; // -30 (Gauche) à +30 (Droite)
  
  if (lookDir == 1 && dist < 70 && dist > 0) steeringBias = -20; // Obstacle Gauche -> Tourne Droite
  else if (lookDir == 2 && dist < 70 && dist > 0) steeringBias = 20; // Obstacle Droite -> Tourne Gauche

  // 3. Application servo
  int angleReelServo = setAckermannAngle(robot, steeringBias, robot.speedAvg);
  robot.currentSteeringBias = steeringBias;

  // 4. Calcul Différentiel Moteur
  // On envoie la vitesse cible globale, et la fonction répartit sur A et B
  calculateDifferentialDrive(robot, robot.speedAvg, steeringBias, targetA, targetB);
}

inline void handleSentryModeState(Robot& robot) {
  switch(robot.sentryState) {
    case SENTRY_IDLE: {
      bool pirState = digitalRead(PIR);
      if (pirState == HIGH && robot.lastPIRState == LOW) {
        if (DEBUG_MODE) Serial.println("SENTRY: PIR Triggered! Starting scan.");
        robot.sentryState = SENTRY_SCAN_START;
      }
      robot.lastPIRState = pirState;
      break;
    }

    case SENTRY_SCAN_START:
      robot.currentScanAngleH = SCAN_H_START_ANGLE;
      tourelle.write(robot.currentScanAngleH, robot.servoNeutralTurret - robot.currentPitch);
      robot.lastActionTime = millis();
      robot.sentryState = SENTRY_SCAN_STEP;
      break;

    case SENTRY_SCAN_STEP:
      if (millis() - robot.lastActionTime > SENTRY_SCAN_SPEED_MS) {
        robot.lastActionTime = millis();
        
        if (robot.distanceLaser > 0 && robot.distanceLaser < SENTRY_DETECTION_RANGE_CM) {
          if (DEBUG_MODE) {
            Serial.print("SENTRY: Intruder detected at angle ");
            Serial.print(robot.currentScanAngleH);
            Serial.print(" and distance ");
            Serial.println(robot.distanceLaser);
          }
          robot.intruderAngle = robot.currentScanAngleH;
          robot.sentryState = SENTRY_TRACKING;
          robot.actionStarted = false; // Reset for tracking state
          return;
        }

        robot.currentScanAngleH += SCAN_H_STEP;
        if (robot.currentScanAngleH > SCAN_H_END_ANGLE) {
          if (DEBUG_MODE) Serial.println("SENTRY: Scan complete, no intruder found.");
          tourelle.write(robot.servoNeutralTurret, robot.servoNeutralTurret - robot.currentPitch);
          robot.sentryState = SENTRY_IDLE; // False alarm, go back to idle
        } else {
          tourelle.write(robot.currentScanAngleH, robot.servoNeutralTurret - robot.currentPitch);
        }
      }
      break;

    case SENTRY_TRACKING:
      if (!robot.actionStarted) {
        robot.actionStarted = true;
        tourelle.write(robot.intruderAngle, robot.servoNeutralTurret - robot.currentPitch);
        PhareAllume();
        robot.lastActionTime = millis();
        if (DEBUG_MODE) Serial.println("SENTRY: Tracking intruder.");
      }
      break;
  }
}

inline void handleCheckingGroundState(Robot& robot, int& pwmA, int& pwmB) {
  // Keep moving forward while checking for cliffs
  pwmA = robot.vitesseCible;
  pwmB = robot.vitesseCible;
  switch(robot.groundCheckState) {
    case GC_START:
      // Arret(); // Don't stop motors for a stable measurement, check while moving
      robot.groundCheckState = GC_LOOK_DOWN;
      break;
    case GC_LOOK_DOWN:
      tourelle.write(tourelle.getAngleHorizontal(), robot.servoAngleGround);
      robot.turretMoveStartTime = millis();
      robot.groundCheckState = GC_WAIT;
      break;
    case GC_WAIT:
      if (millis() - robot.turretMoveStartTime > robot.turretMoveTime) {
        robot.groundCheckState = GC_CHECK;
      }
      break;
    case GC_CHECK:
      // Distance Laser is updated in the main loop
      if (robot.distanceLaser > robot.seuilVide || robot.distanceLaser == 0) { // distance 0 can be an error/out of range
        changeState(robot, CLIFF_DETECTED);
      } else {
        robot.groundCheckState = GC_LOOK_UP;
      }
      break;
    case GC_LOOK_UP:
      tourelle.write(tourelle.getAngleHorizontal(), robot.servoNeutralTurret - robot.currentPitch);
      robot.turretMoveStartTime = millis();
      robot.groundCheckState = GC_FINISH;
      break;
    case GC_FINISH:
      if (millis() - robot.turretMoveStartTime > robot.turretMoveTime) {
        robot.lastCliffCheckTime = millis();
        changeState(robot, robot.stateBeforeGroundCheck); // Return to previous state
      }
      break;
  }
}

inline void handleCliffDetectedState(Robot& robot, int& pwmA, int& pwmB) {
  // Emergency maneuver
  setLcdText(robot, "ATTENTION VIDE !");
  if (!robot.actionStarted) {
      robot.actionStarted = true;
      robot.lastActionTime = millis();
      Arret();
  }
  if (millis() - robot.lastActionTime < robot.avoidBackupDuration) {
      pwmA = -robot.speedSlow;
      pwmB = -robot.speedSlow;
  } else {
      changeState(robot, IDLE);
  }
}

inline void handleAnimatingHeadState(Robot& robot) {
  // Stop motors during animation
  motorA.motorGo(0);
  motorB.motorGo(0);
  handleHeadAnimation(robot, true);
}


// Prototype for applyTractionControl
inline void applyTractionControl(int& pwmA, int& pwmB, Robot& robot);

// --- UPDATE MOTOR CONTROL (LE CERVEAU CENTRAL) ---

inline void applyTractionControl(int& pwmA, int& pwmB, Robot& robot) {
    // Implemente un contrôle de traction simplifié.
    // Si le robot est confirmé comme étant bloqué (isStuckConfirmed), réduit la puissance des moteurs.
    // Ceci aide à retrouver de l'adhérence si le robot patine contre un obstacle ou sur une surface glissante.
    if (robot.isStuckConfirmed) {
        pwmA *= 0.6;
        pwmB *= 0.6;
    }
}

inline void updateMotorControl(Robot& robot) {
  // Le cœur du système de contrôle moteur. Cette fonction est appelée à chaque itération de la boucle principale.
  // Elle est responsable de :
  // 1. Déterminer les vitesses cibles (targetA, targetB) pour chaque moteur en fonction de l'état actuel du robot.
  // 2. Appliquer un lissage (ramping) aux vitesses cibles pour des accélérations et décélérations douces.
  // 3. Intégrer les mécanismes de "Power Steering" et de "Traction Control" pour une conduite plus sophistiquée.
  // 4. Envoyer les commandes PWM finales aux moteurs.
  // Gère également les arrêts d'urgence (batterie critique) et la détection d'obstacles/blocages.
  if (robot.batteryIsCritical) { Arret(); return; }

  // Variables Cibles (ce qu'on VEUT atteindre)
  int targetA = 0;
  int targetB = 0;

  // Variables Statiques (Vitesse actuelle pour le ramping)
  static float currentPwmA = 0;
  static float currentPwmB = 0;

  // 1. Déterminer la cible selon l'état
  switch (robot.currentState) {
    case IDLE: targetA = 0; targetB = 0; break;
    case MOVING_FORWARD: handleMovingForwardState(robot, targetA, targetB); break;
    case SMART_AVOIDANCE: handleSmartAvoidanceState(robot, targetA, targetB); break;
    case FOLLOW_HEADING: handleFollowHeadingState(robot, targetA, targetB); break;
    
    // Cas simples sans différentiel complexe
    case MOVING_BACKWARD: targetA = -robot.vitesseCible; targetB = -robot.vitesseCible; break;
    case MANUAL_BACKWARD: targetA = -robot.vitesseCible; targetB = -robot.vitesseCible; break;
    case MANUAL_FORWARD: targetA = robot.vitesseCible; targetB = robot.vitesseCible; break;
    case MANUAL_TURNING_LEFT: targetA = robot.speedRotation; targetB = -robot.speedRotation; break;
    case MANUAL_TURNING_RIGHT: targetA = -robot.speedRotation; targetB = robot.speedRotation; break;

    case OBSTACLE_AVOIDANCE: handleObstacleAvoidanceState(robot, targetA, targetB); break;
    case EMERGENCY_EVASION: handleEmergencyEvasionState(robot, targetA, targetB); break;
    case STUCK: handleStuckState(robot, targetA, targetB); break;
    
    case CHECKING_GROUND: targetA = robot.vitesseCible; targetB = robot.vitesseCible; break;
    case CLIFF_DETECTED: 
         if (!robot.actionStarted) { robot.actionStarted = true; robot.lastActionTime = millis(); targetA=0; targetB=0; }
         else if (millis() - robot.lastActionTime < robot.avoidBackupDuration) { targetA = -robot.speedSlow; targetB = -robot.speedSlow; } 
         else { changeState(robot, IDLE); }
         break;
    case SENTRY_MODE: handleSentryModeState(robot); break;
    case ANIMATING_HEAD: handleAnimatingHeadState(robot); break;
    case GAMEPAD_CONTROL: 
        // PWMs set directly by gamepad handler, no ramping or differential applied here
        targetA = motorA.getPWM(); 
        targetB = motorB.getPWM(); 
        break;

    default: targetA = 0; targetB = 0; break;
  }

  // --- Power Steering (Surplus de puissance en virage serré) ---
  if (abs(robot.currentSteeringBias) > 15) { 
      targetA *= 1.15; 
      targetB *= 1.15;
  }

  // 2. Application du Ramping (Lissage)
  // On ne lisse PAS si on est en arrêt d'urgence ou évasion (besoin de réaction immédiate)
  if (robot.currentState == EMERGENCY_EVASION || robot.currentState == STUCK || robot.currentState == CLIFF_DETECTED) {
      currentPwmA = targetA;
      currentPwmB = targetB;
  } else {
      currentPwmA = applySpeedRamp(robot, targetA, currentPwmA);
      currentPwmB = applySpeedRamp(robot, targetB, currentPwmB);
  }

  // 3. Envoi au Hardware
  int finalA = (int)currentPwmA;
  int finalB = (int)currentPwmB;

  // Calibration et Inversion
  finalB = (int)(finalB * robot.motorBCalibration);
  if (robot.controlInverted) { finalA *= -1; finalB *= -1; }

  // Zone morte (si PWM trop faible pour bouger, on coupe pour ne pas faire chauffer)
  if (abs(finalA) < MIN_SPEED_TO_MOVE && targetA == 0) finalA = 0;
  if (abs(finalB) < MIN_SPEED_TO_MOVE && targetB == 0) finalB = 0;

  // 4. Application du Traction Control
  applyTractionControl(finalA, finalB, robot);

  motorA.motorGo(constrain(finalA, -PWM_MAX, PWM_MAX));
  motorB.motorGo(constrain(finalB, -PWM_MAX, PWM_MAX));
}

inline void handleEmergencyEvasionState(Robot& robot, int& pwmA, int& pwmB) {
  // "Nono-Escape" maneuver
  switch(robot.evasionState) {
    case EVADE_START:
      Arret();
      // Play a sound, blink lights, etc.
      // lcd.setRGB(255, 0, 0);
      // setLcdText(robot, "Bumper Hit!");
      robot.actionStarted = false;
      robot.evasionState = EVADE_BACKUP;
      break;

    case EVADE_BACKUP:
      if (!robot.actionStarted) {
        robot.actionStarted = true;
        robot.lastActionTime = millis();
        if (DEBUG_MODE) Serial.println("EVADE: Backing up...");
      }

      if (millis() - robot.lastActionTime < 800) { // Back up for 800ms
        pwmA = -robot.speedAvg;
        pwmB = -robot.speedAvg;
      } else {
        Arret();
        robot.actionStarted = false;
        robot.evasionState = EVADE_PIVOT;
      }
      break;

    case EVADE_PIVOT: { // Added scope for error variable
      if (!robot.actionStarted) {
        // Here you could add a laser scan to find the best escape direction.
        // For now, we pivot 90 degrees right.
        if (DEBUG_MODE) Serial.println("EVADE: Pivoting...");
        robot.capCibleRotation = getCalibratedHeading(robot) + 90.0;
        if (robot.capCibleRotation >= 360) robot.capCibleRotation -= 360;
        robot.actionStarted = true;
      }
      
      float error = calculateHeadingError(robot.capCibleRotation, getCalibratedHeading(robot));
      if (abs(error) > robot.turnTolerance) {
        pwmA = -robot.speedRotation; // Pivot right
        pwmB = robot.speedRotation;
      } else {
        robot.evasionState = EVADE_FINISH;
      }
      break;
    } // End of scope for error variable

    case EVADE_FINISH:
      if (DEBUG_MODE) Serial.println("EVADE: Maneuver complete.");
      Arret();
      robot.evasionState = EVADE_START; // Reset for next time
      changeState(robot, OBSTACLE_AVOIDANCE); // After evading, assess the situation
      break;
  }
}

inline void handleStuckState(Robot& robot, int& pwmA, int& pwmB) {
  // A simplified version of the evasion maneuver
  if (!robot.actionStarted) {
    robot.actionStarted = true;
    robot.lastActionTime = millis();
    if (DEBUG_MODE) Serial.println("STUCK: Attempting to un-stuck...");
    // setLcdText(robot, "I'm stuck!");
    Arret();
  }

  // Back up, then let the standard obstacle avoidance take over.
  if (millis() - robot.lastActionTime < 1000) {
    pwmA = -robot.speedAvg;
    pwmB = -robot.speedAvg;
  } else {
    changeState(robot, OBSTACLE_AVOIDANCE);
  }
}

inline float calculateHeadingError(float target, float current) {
  float error = target - current;
  if (error > 180) error -= 360;
  else if (error < -180) error += 360;
  return error;
}

#endif // FONCTIONS_MOTRICES_H