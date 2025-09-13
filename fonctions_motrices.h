#ifndef FONCTIONS_MOTRICES_H
#define FONCTIONS_MOTRICES_H

#define SERVO_TURN_OFFSET 20
#define PIR_TURN_SPEED 50

// === FONCTIONS DE BASE ===

void changeState(RobotState newState) {
  // Validate state transition
  if (newState < 0 || newState > SMART_TURNING) {
    if (DEBUG_MODE) {
      Serial.print("ERREUR: État invalide: ");
      Serial.println(newState);
    }
    return;
  }
  
  RobotState previousState = currentState;
  currentState = newState;
  actionStarted = false;
  lastActionTime = millis();
  
  // Réinitialiser les variables d'évitement si on sort du mode AVOID_MANEUVER
  if (newState != AVOID_MANEUVER) {
    hasReculed = false;
    hasTurned = false;
  }
  
  // Log state transition
  if (DEBUG_MODE) {
    Serial.print("Transition d'état: ");
    Serial.print(previousState);
    Serial.print(" -> ");
    Serial.println(newState);
  }
}

// Fonction pour calculer l'erreur de cap (gère le passage par 360°)
float calculateHeadingError(float target, float current) {
  float error = target - current;
  
  // Gérer le passage par 360°
  if (error > 180) {
    error -= 360;
  } else if (error < -180) {
    error += 360;
  }
  
  return error;
}

// Fonction pour calculer le cap cible après un virage
float calculateTargetHeading(float currentHeading, float turnAngle) {
  float target = currentHeading + turnAngle;
  
  // Gérer le passage par 0°/360°
  if (target >= 360) {
    target -= 360;
  }
  if (target < 0) {
    target += 360;
  }
  return target;
}

// Fonction d'arrêt
void Arret() {
  vitesseCible = 0;
  vitesseCourante = 0;
  motorA.motorBrake(255);
  motorB.motorBrake(255);
  motorA.motorStop();
  motorB.motorStop();
}

// === NOUVELLE FONCTION DE CONTRÔLE MOTEUR CENTRALISÉE ===

void updateMotorControl() {
  // Sécurité d'abord : vérifier les obstacles
  if (dusm < darret || digitalRead(INTERUPTPIN) == LOW) {
    Arret();
    if (currentState == OBSTACLE_AVOIDANCE) {
      changeState(AVOID_MANEUVER);
    } else if (currentState == MOVING_FORWARD) {
      changeState(IDLE);
    }
    return;
  }
  
  // Gestion de la rampe d'accélération/décélération
  if (vitesseCourante < vitesseCible) {
    vitesseCourante = min(vitesseCourante + RAMP_STEP, vitesseCible);
  } else if (vitesseCourante > vitesseCible) {
    vitesseCourante = max(vitesseCourante - RAMP_STEP, vitesseCible);
  }
  
  // Variables pour les commandes PWM finales
  int pwmA = 0;
  int pwmB = 0;
  
  // Logique par état
  switch (currentState) {
    case MOVING_FORWARD:
      pwmA = vitesseCourante;
      pwmB = vitesseCourante;
      Servodirection.write(neutredirection);
      break;
      
    case MOVING_BACKWARD:
      pwmA = -vitesseCourante;
      pwmB = -vitesseCourante;
      Servodirection.write(neutredirection);
      break;
      
    case MANUAL_MOVING_FORWARD:
      pwmA = vitesseCourante;
      pwmB = vitesseCourante;
      Servodirection.write(neutredirection);
      
      // Vérifier si le mouvement de distance est terminé
      if (checkManualDistanceComplete()) {
        // Le mouvement est terminé, on reste en IDLE
        return;
      }
      break;
      
    case MANUAL_MOVING_BACKWARD:
      pwmA = -vitesseCourante;
      pwmB = -vitesseCourante;
      Servodirection.write(neutredirection);
      
      // Vérifier si le mouvement de distance est terminé
      if (checkManualDistanceComplete()) {
        // Le mouvement est terminé, on reste en IDLE
        return;
      }
      break;
      
         case TURNING_LEFT:
       pwmA = -vitesseCourante;
       pwmB = vitesseCourante;
       Servodirection.write(neutredirection + SERVO_TURN_OFFSET);
       
       // La lecture du compas est maintenant faite dans sensor_update_task()
       float currentHeading = getCalibratedHeading(compass);
       float error = calculateHeadingError(capCibleRotation, currentHeading);
       
       if (abs(error) < TOLERANCE_VIRAGE) {
         if (DEBUG_MODE) {
           Serial.print("Virage Gauche TERMINE. Cap atteint: ");
           Serial.println(currentHeading);
         }
         changeState(IDLE);
       }
       break;
       
           case TURNING_RIGHT:
        pwmA = vitesseCourante;
        pwmB = -vitesseCourante;
        Servodirection.write(neutredirection + SERVO_TURN_OFFSET);
        
        // La lecture du compas est maintenant faite dans sensor_update_task()
        float currentHeadingRight = getCalibratedHeading(compass);
        float errorRight = calculateHeadingError(capCibleRotation, currentHeadingRight);
        
        if (abs(errorRight) < TOLERANCE_VIRAGE) {
          if (DEBUG_MODE) {
            Serial.print("Virage Droite TERMINE. Cap atteint: ");
            Serial.println(currentHeadingRight);
          }
          changeState(IDLE);
        }
        break;
        
    case MANUAL_TURNING_LEFT:
      // Virage simple en mode manuel - pas de vérification de cap
      pwmA = -vitesseCourante;
      pwmB = vitesseCourante;
      Servodirection.write(neutredirection + SERVO_TURN_OFFSET);
      // L'utilisateur doit envoyer "stop" pour arrêter
      break;
      
    case MANUAL_TURNING_RIGHT:
      // Virage simple en mode manuel - pas de vérification de cap
      pwmA = vitesseCourante;
      pwmB = -vitesseCourante;
      Servodirection.write(neutredirection + SERVO_TURN_OFFSET);
      // L'utilisateur doit envoyer "stop" pour arrêter
      break;
      
    case OBSTACLE_AVOIDANCE:
      pwmA = vitesseCourante;
      pwmB = vitesseCourante;
      Servodirection.write(neutredirection);
      break;
      
          case FOLLOW_HEADING:
        // Contrôle de cap avec correction proportionnelle
        // La lecture du compas est maintenant faite dans sensor_update_task()
        float currentHeadingFollow = getCalibratedHeading(compass);
        float errorFollow = calculateHeadingError(Ncap, currentHeadingFollow);
        
        // Contrôleur proportionnel
        int adjustment = Kp_HEADING * errorFollow;
        adjustment = constrain(adjustment, -vitesseCourante/2, vitesseCourante/2);
        
        pwmA = vitesseCourante + adjustment;
        pwmB = vitesseCourante - adjustment;
        
        // Correction du servo de direction pour plus de réactivité
        int servoAdjustment = constrain(errorFollow * SERVO_ADJUSTMENT_FACTOR, -SERVO_MAX_ADJUSTMENT, SERVO_MAX_ADJUSTMENT);
        Servodirection.write(neutredirection + servoAdjustment);
        break;
        
    case MAINTAIN_HEADING:
        // Arrêt avec maintien du cap - correction de direction sans avancer
        // La lecture du compas est maintenant faite dans sensor_update_task()
        float currentHeadingMaintain = getCalibratedHeading(compass);
        float errorMaintain = calculateHeadingError(Ncap, currentHeadingMaintain);
        
        // Correction de direction uniquement (pas de mouvement)
        pwmA = 0;
        pwmB = 0;
        
        // Correction du servo de direction pour maintenir l'orientation
        int servoAdjustmentMaintain = constrain(errorMaintain * SERVO_ADJUSTMENT_FACTOR, -SERVO_MAX_ADJUSTMENT, SERVO_MAX_ADJUSTMENT);
        Servodirection.write(neutredirection + servoAdjustmentMaintain);
        break;
      
    case PIR_DETECT:
      // Mode détection PIR - arrêt par défaut
      pwmA = 0;
      pwmB = 0;
      if (digitalRead(PIR) == LOW) {
        Serial.println("PIR: mouvement détecté");
        PhareAllume();
        // Tourner lentement sur place
        pwmA = -PIR_TURN_SPEED;
        pwmB = PIR_TURN_SPEED;
      }
      break;
      
    case AVOID_MANEUVER:
      // Manœuvre d'évitement - reculer puis tourner
      if (!hasReculed) {
        pwmA = -vitesseCourante;
        pwmB = -vitesseCourante;
        hasReculed = true;
      } else if (!hasTurned) {
        // Tourner aléatoirement
        if (random(TURN_LEFT_CHOICE, TURN_RIGHT_CHOICE) == TURN_LEFT_CHOICE) {
          pwmA = -vitesseCourante;
          pwmB = vitesseCourante;
        } else {
          pwmA = vitesseCourante;
          pwmB = -vitesseCourante;
        }
        hasTurned = true;
      } else {
        // Manœuvre terminée
        hasReculed = false;
        hasTurned = false;
        vitesseCible = VITESSE_LENTE;
        changeState(OBSTACLE_AVOIDANCE);
      }
      break;
      
    default:
      // État IDLE ou autres états - arrêt
      pwmA = 0;
      pwmB = 0;
      if (DEBUG_MODE) {
        Serial.print("ATTENTION: État inconnu dans updateMotorControl(): ");
        Serial.println(currentState);
      }
      break;
  }
  
  // Calibration du moteur B
  pwmB = (int)(pwmB * CALIBRATION_MOTEUR_B);
  
  // Limitation des valeurs PWM
  pwmA = constrain(pwmA, -255, 255);
  pwmB = constrain(pwmB, -255, 255);
  
  // Commande finale aux moteurs
  motorA.motorGo(pwmA);
  motorB.motorGo(pwmB);
}

// === FONCTIONS DE DÉBOGAGE ET CALIBRATION DU COMPAS ===

// Fonction pour obtenir le cap calibré (avec correction 360°)
float getCalibratedHeading(LSM303 &compass) {
  // Vérifier que le compas a été initialisé
  if (!compassInitialized) {
    if (DEBUG_MODE) Serial.println("ATTENTION: Compas non initialise dans getCalibratedHeading()");
    return 0.0; // Retourner 0 en cas d'erreur
  }
  
  // Vérifier que le compas a été lu avec succès
  if (compass.m.x == 0 && compass.m.y == 0 && compass.m.z == 0) {
    if (DEBUG_MODE) Serial.println("ATTENTION: Valeurs compas nulles dans getCalibratedHeading()");
    return 0.0; // Retourner 0 en cas d'erreur
  }

  // Appliquer la correction de calibration si le compas est calibré
  if (compassCalibrated) {
    // Hard-iron correction (offset)
    float corrected_x = compass.m.x - (magMin.x + magMax.x) / 2.0;
    float corrected_y = compass.m.y - (magMin.y + magMax.y) / 2.0;

    // Soft-iron correction (scaling)
    float avg_delta_x = (magMax.x - magMin.x) / 2.0;
    float avg_delta_y = (magMax.y - magMin.y) / 2.0;
    
    // Avoid division by zero if min/max are the same (no rotation during calibration)
    if (avg_delta_x == 0 || avg_delta_y == 0) {
        if (DEBUG_MODE) Serial.println("ATTENTION: Calibration min/max invalide (division par zero)");
        return calculateHeading(compass); // Fallback to raw heading
    }

    float avg_delta = (avg_delta_x + avg_delta_y) / 2.0;
    float scaled_x = corrected_x * (avg_delta / avg_delta_x);
    float scaled_y = corrected_y * (avg_delta / avg_delta_y);

    // Calculate heading from corrected and scaled values
    float heading = (atan2(scaled_y, scaled_x) * 180) / PI;
    if (heading < 0) {
      heading += 360;
    }
    
    // Apply compassInverted correction if needed (from original code)
    if (compassInverted) {
      heading += 180.0;
      if (heading >= 360.0) {
        heading -= 360.0;
      }
    }
    return heading;

  } else {
    // If not calibrated, return raw heading (or apply simple offset if that's desired fallback)
    // For now, returning raw heading if not 360-degree calibrated
    if (DEBUG_MODE) Serial.println("Compas non calibre (360 deg) - Retourne cap brut.");
    return calculateHeading(compass); // Use the existing calculateHeading for raw
  }
}

// Fonction de calibration du compas (360 degrés)
void calibrateCompass() {
  static unsigned long startTime = 0;
  const unsigned long CALIBRATION_DURATION_MS = 15000; // 15 seconds for rotation
  static bool calibrationStarted = false;

  if (!calibrationStarted) {
    Serial.println("=== CALIBRATION DU COMPAS (360 DEGRES) ===");
    Serial.println("Faites pivoter le robot LENTEMENT sur tous les axes (X, Y, Z) pendant 15 secondes.");
    Serial.println("Le robot va collecter les valeurs min/max du champ magnétique.");
    Serial.println("Calibration en cours...");
    
    // Reset min/max values to their extremes
    magMin = {32767, 32767, 32767};
    magMax = {-32768, -32768, -32768};
    
    startTime = millis();
    calibrationStarted = true;
    
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Calib 360 deg");
    lcd.setCursor(0,1);
    lcd.print("Rotation...");
  }

  // Collect min/max values during the calibration duration
  if (millis() - startTime < CALIBRATION_DURATION_MS) {
    compass.read(); // Read raw magnetometer values
    
    magMin.x = min(magMin.x, compass.m.x);
    magMin.y = min(magMin.y, compass.m.y);
    magMin.z = min(magMin.z, compass.m.z);

    magMax.x = max(magMax.x, compass.m.x);
    magMax.y = max(magMax.y, compass.m.y);
    magMax.z = max(magMax.z, compass.m.z);
    
    // Optional: Print current min/max to serial for user feedback
    if (DEBUG_MODE && millis() % 1000 < 100) { // Print every second
      Serial.print("Min: {"); Serial.print(magMin.x); Serial.print(", "); Serial.print(magMin.y); Serial.print(", "); Serial.print(magMin.z); Serial.print("} ");
      Serial.print("Max: {"); Serial.print(magMax.x); Serial.print(", "); Serial.print(magMax.y); Serial.print(", "); Serial.print(magMax.z); Serial.println("}");
    }
  } else {
    // Calibration finished
    Serial.println("=== CALIBRATION TERMINEE ===");
    Serial.println("Resultats finaux:");
    Serial.print("Min: {"); Serial.print(magMin.x); Serial.print(", "); Serial.print(magMin.y); Serial.print(", "); Serial.print(magMin.z); Serial.println("}");
    Serial.print("Max: {"); Serial.print(magMax.x); Serial.print(", "); Serial.print(magMax.y); Serial.print(", "); Serial.print(magMax.z); Serial.println("}");

    // Check if min/max values have changed significantly from initial extremes
    // This is a basic check to see if rotation actually occurred
    if (magMin.x < 32000 && magMax.x > -32000 &&
        magMin.y < 32000 && magMax.y > -32000 &&
        magMin.z < 32000 && magMax.z > -32000) {
      compassCalibrated = true;
      Serial.println("✅ Calibration REUSSIE - Plage de valeurs couverte.");
    } else {
      compassCalibrated = false;
      Serial.println("⚠️ Calibration DEFAILLANTE - Plage de valeurs insuffisante.");
      Serial.println("Assurez-vous de faire pivoter le robot sur tous les axes.");
    }

    // Afficher sur l'écran LCD
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Calib terminee");
    lcd.setCursor(0,1);
    if (compassCalibrated) {
      lcd.print("OK");
    } else {
      lcd.print("ECHEC");
    }

    // Sauvegarder automatiquement en EEPROM
    saveCompassCalibration();
    calibrationStarted = false; // Reset for next time
  }
}

// Fonction de débogage du compas
void debugCompass() {
  Serial.println("=== DÉBOGAGE DU COMPAS ===");
  
  // Lire les valeurs brutes
  compass.read();
  
  Serial.print("Magnétomètre brut - X: ");
  Serial.print(compass.m.x);
  Serial.print(" Y: ");
  Serial.print(compass.m.y);
  Serial.print(" Z: ");
  Serial.println(compass.m.z);
  
  // Calculer la magnitude du champ magnétique
  float magnitude = sqrt(compass.m.x * compass.m.x + 
                        compass.m.y * compass.m.y + 
                        compass.m.z * compass.m.z);
  
  Serial.print("Magnitude du champ magnétique: ");
  Serial.println(magnitude);
  
  // Lire l'accéléromètre
  Serial.print("Accéléromètre - X: ");
  Serial.print(compass.a.x);
  Serial.print(" Y: ");
  Serial.print(compass.a.y);
  Serial.print(" Z: ");
  Serial.println(compass.a.z);
  
  // Calculer l'inclinaison
  float pitch = atan2(compass.a.y, sqrt(compass.a.x * compass.a.x + compass.a.z * compass.a.z)) * 180 / PI;
  float roll = atan2(compass.a.x, compass.a.z) * 180 / PI;
  
  Serial.print("Inclinaison - Pitch: ");
  Serial.print(pitch);
  Serial.print("° Roll: ");
  Serial.print(roll);
  Serial.println("°");
  
  // Afficher le cap brut et calibré
  float rawHeading = calculateHeading(compass);
  float calibratedHeading = getCalibratedHeading(compass);
  
  Serial.print("Cap brut: ");
  Serial.print(rawHeading);
  Serial.print("° | Cap calibré: ");
  Serial.print(calibratedHeading);
  Serial.println("°");
  
  // Vérifier la qualité du signal
  if (magnitude < 100) {
    Serial.println("⚠️ ATTENTION: Signal magnétique faible");
  } else if (magnitude > 1000) {
    Serial.println("⚠️ ATTENTION: Signal magnétique très fort (interférence?)");
  } else {
    Serial.println("✅ Signal magnétique normal");
  }
  
  // Afficher sur l'écran LCD
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Cap: ");
  lcd.print((int)calibratedHeading);
  lcd.print(" deg");
  lcd.setCursor(0,1);
  lcd.print("Mag: " + String((int)magnitude));
}

// Fonction pour afficher les informations du compas
void displayCompassInfo() {
  compass.read();
  
  // Vérifier si les valeurs du compas sont valides
  if (compass.m.x == 0 && compass.m.y == 0 && compass.m.z == 0) {
    if (DEBUG_MODE) Serial.println("ERREUR: Valeurs compas nulles dans displayCompassInfo()");
    lcd.clear();
    lcd.print("ERREUR COMPAS");
    lcd.setCursor(0,1);
    lcd.print("Valeurs nulles");
    return;
  }
  
  float heading = getCalibratedHeading(compass);
  
  // Déterminer la direction cardinale
  String direction;
  if (heading >= 337.5 || heading < 22.5) direction = "NORD";
  else if (heading >= 22.5 && heading < 67.5) direction = "NE";
  else if (heading >= 67.5 && heading < 112.5) direction = "EST";
  else if (heading >= 112.5 && heading < 157.5) direction = "SE";
  else if (heading >= 157.5 && heading < 202.5) direction = "SUD";
  else if (heading >= 202.5 && heading < 247.5) direction = "SO";
  else if (heading >= 247.5 && heading < 292.5) direction = "OUEST";
  else direction = "NO";
  
  // Afficher sur l'écran LCD
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(direction);
  lcd.print(" ");
  lcd.print((int)heading);
  lcd.print(" deg");
  lcd.setCursor(0,1);
  lcd.print("Cal: ");
  lcd.print(compassCalibrated ? "OK" : "NON");
  
  // Afficher dans le moniteur série
  if (DEBUG_MODE) {
    Serial.print("Direction: ");
    Serial.print(direction);
    Serial.print(" | Cap: ");
    Serial.print(heading);
    Serial.print("° | Calibré: ");
    Serial.println(compassCalibrated ? "OUI" : "NON");
  }
}

// === FONCTIONS DE GESTION EEPROM POUR LA CALIBRATION ===

// Vérifier si les données EEPROM sont valides
bool isEEPROMDataValid() {
  int magicValue = 0;
  EEPROM.get(EEPROM_MAGIC_NUMBER_ADDR, magicValue);
  return (magicValue == EEPROM_MAGIC_VALUE);
}

// Sauvegarder la calibration du compas en EEPROM
void saveCompassCalibration() {
  if (DEBUG_MODE) Serial.println("Sauvegarde de la calibration en EEPROM...");
  
  // Sauvegarder le magic number pour vérifier l'intégrité
  EEPROM.put(EEPROM_MAGIC_NUMBER_ADDR, EEPROM_MAGIC_VALUE);

  // Sauvegarder l'offset de calibration (still keep it for backward compatibility or if needed)
  EEPROM.put(EEPROM_COMPASS_OFFSET_ADDR, compassOffset);
  
  // Sauvegarder le statut de calibration
  EEPROM.put(EEPROM_COMPASS_CALIBRATED_ADDR, compassCalibrated);

  // Sauvegarder les valeurs min/max du magnétomètre
  EEPROM.put(EEPROM_MAG_MIN_X_ADDR, magMin.x);
  EEPROM.put(EEPROM_MAG_MIN_Y_ADDR, magMin.y);
  EEPROM.put(EEPROM_MAG_MIN_Z_ADDR, magMin.z);
  EEPROM.put(EEPROM_MAG_MAX_X_ADDR, magMax.x);
  EEPROM.put(EEPROM_MAG_MAX_Y_ADDR, magMax.y);
  EEPROM.put(EEPROM_MAG_MAX_Z_ADDR, magMax.z);
  
  if (DEBUG_MODE) Serial.println("Calibration sauvegardee avec succes!");
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Calib sauvegardee");
}

// Charger la calibration du compas depuis l'EEPROM
void loadCompassCalibration() {
  if (DEBUG_MODE) Serial.println("Chargement de la calibration depuis l'EEPROM...");
  
  if (isEEPROMDataValid()) {
    // Charger l'offset de calibration
    EEPROM.get(EEPROM_COMPASS_OFFSET_ADDR, compassOffset);
    
    // Charger le statut de calibration
    EEPROM.get(EEPROM_COMPASS_CALIBRATED_ADDR, compassCalibrated);

    // Charger les valeurs min/max du magnétomètre
    EEPROM.get(EEPROM_MAG_MIN_X_ADDR, magMin.x);
    EEPROM.get(EEPROM_MAG_MIN_Y_ADDR, magMin.y);
    EEPROM.get(EEPROM_MAG_MIN_Z_ADDR, magMin.z);
    EEPROM.get(EEPROM_MAG_MAX_X_ADDR, magMax.x);
    EEPROM.get(EEPROM_MAG_MAX_Y_ADDR, magMax.y);
    EEPROM.get(EEPROM_MAG_MAX_Z_ADDR, magMax.z);
    
    if (DEBUG_MODE) {
      Serial.print("Calibration chargee - Offset: ");
      Serial.print(compassOffset);
      Serial.print(" deg, Calibre: ");
      Serial.println(compassCalibrated ? "OUI" : "NON");
      Serial.print("MagMin: {"); Serial.print(magMin.x); Serial.print(", "); Serial.print(magMin.y); Serial.print(", "); Serial.print(magMin.z); Serial.println("}");
      Serial.print("MagMax: {"); Serial.print(magMax.x); Serial.print(", "); Serial.print(magMax.y); Serial.print(", "); Serial.print(magMax.z); Serial.println("}");
    }
    
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Calib chargee");
    lcd.setCursor(0,1);
    lcd.print("Offset: ");
    lcd.print((int)compassOffset);
    lcd.print(" deg");
  } else {
    if (DEBUG_MODE) Serial.println("Aucune calibration valide trouvee en EEPROM");
    compassOffset = 0.0;
    compassCalibrated = false;
    // Reset min/max to default extremes if no valid data
    magMin = {32767, 32767, 32767};
    magMax = {-32768, -32768, -32768};
    
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Pas de calib");
    lcd.setCursor(0,1);
    lcd.print("en EEPROM");
  }
}

// === FONCTIONS DE VIRAGE AMÉLIORÉES ===

// Calculer la vitesse de virage en fonction de l'erreur angulaire
float calculateTurnSpeed(float error) {
  float absError = abs(error);
  
  if (absError <= PRECISE_TURN_TOLERANCE) {
    return 0; // Arrêt si on est assez proche
  } else if (absError <= PRECISE_TURN_SLOWDOWN_ANGLE) {
    // Ralentissement progressif quand on approche de la cible
    float speedRatio = absError / PRECISE_TURN_SLOWDOWN_ANGLE;
    return PRECISE_TURN_MIN_SPEED + (PRECISE_TURN_MAX_SPEED - PRECISE_TURN_MIN_SPEED) * speedRatio;
  } else {
    // Vitesse maximale pour les grandes erreurs
    return PRECISE_TURN_MAX_SPEED;
  }
}

// Exécuter un virage précis avec ralentissement progressif
void executePreciseTurn(float targetAngle) {
  if (!compassInitialized) {
    if (DEBUG_MODE) Serial.println("ERREUR: Compas non initialise pour virage precis");
    showErrorCode(ERROR_COMPASS_READ);
    return;
  }
  
  compass.read();
  float currentHeading = getCalibratedHeading(compass);
  float error = calculateHeadingError(targetAngle, currentHeading);
  
  if (DEBUG_MODE) {
    Serial.print("Virage precis - Erreur: ");
    Serial.print(error);
    Serial.print(" deg, Vitesse calculee: ");
  }
  
  float turnSpeed = calculateTurnSpeed(error);
  if (DEBUG_MODE) Serial.println(turnSpeed);
  
  if (abs(error) <= PRECISE_TURN_TOLERANCE) {
    // Virage terminé
    if (DEBUG_MODE) Serial.println("Virage precis TERMINE!");
    changeState(IDLE);
    vitesseCible = 0;
    
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Virage termine");
    lcd.setCursor(0,1);
    lcd.print("Cap: ");
    lcd.print((int)currentHeading);
    lcd.print(" deg");
  } else {
    // Continuer le virage avec vitesse ajustée
    vitesseCible = turnSpeed;
    
    if (error > 0) {
      // Tourner à droite
      changeState(TURNING_RIGHT);
    } else {
      // Tourner à gauche
      changeState(TURNING_LEFT);
    }
    
    // Affichage sur LCD
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Virage precis");
    lcd.setCursor(0,1);
    lcd.print("Erreur: ");
    lcd.print((int)abs(error));
    lcd.print(" deg");
  }
}

// === FONCTIONS DE CONTRÔLE DE DISTANCE MANUEL ===

// Définir la distance pour les mouvements manuels
void setManualDistance(int distance) {
  manualDistance = constrain(distance, MANUAL_MIN_DISTANCE, MANUAL_MAX_DISTANCE);
  if (DEBUG_MODE) {
    Serial.print("Distance manuelle definie a: ");
    Serial.print(manualDistance);
    Serial.println(" cm");
  }
}

// Exécuter un mouvement de distance précis en mode manuel
void executeManualDistanceMove(bool forward) {
  // Utiliser la dernière distance connue, mise à jour par la tâche de fond
  manualStartDistance = dusm;
  manualMovementActive = true;
  
  if (DEBUG_MODE) {
    Serial.print("Mouvement manuel - Distance de depart: ");
    Serial.print(manualStartDistance);
    Serial.print(" cm, Distance cible: ");
    Serial.print(manualDistance);
    Serial.println(" cm");
  }
  
  if (forward) {
    changeState(MANUAL_MOVING_FORWARD);
    vitesseCible = VITESSE_MOYENNE;
  } else {
    changeState(MANUAL_MOVING_BACKWARD);
    vitesseCible = VITESSE_LENTE;
  }
}

// Vérifier si le mouvement de distance manuel est terminé
bool checkManualDistanceComplete() {
  // Utiliser la dernière distance connue, mise à jour par la tâche de fond
  
  if (currentState == MANUAL_MOVING_FORWARD) {
    // Pour l'avance, on vérifie que la distance parcourue est suffisante
    int distanceTraveled = manualStartDistance - dusm;
    
    if (distanceTraveled >= manualDistance - MANUAL_DISTANCE_TOLERANCE) {
      if (DEBUG_MODE) {
        Serial.print("Avance manuelle TERMINEE - Distance parcourue: ");
        Serial.print(distanceTraveled);
        Serial.println(" cm");
      }
      changeState(IDLE);
      vitesseCible = 0;
      manualMovementActive = false;
      
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Avance terminee");
      lcd.setCursor(0,1);
      lcd.print("Distance: ");
      lcd.print(distanceTraveled);
      lcd.print(" cm");
      return true;
    }
  } else if (currentState == MANUAL_MOVING_BACKWARD) {
    // Pour le recul, on vérifie que la distance parcourue est suffisante
    int distanceTraveled = dusm - manualStartDistance;
    
    if (distanceTraveled >= manualDistance - MANUAL_DISTANCE_TOLERANCE) {
      if (DEBUG_MODE) {
        Serial.print("Recul manuel TERMINE - Distance parcourue: ");
        Serial.print(distanceTraveled);
        Serial.println(" cm");
      }
      changeState(IDLE);
      vitesseCible = 0;
      manualMovementActive = false;
      
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Recul termine");
      lcd.setCursor(0,1);
      lcd.print("Distance: ");
      lcd.print(distanceTraveled);
      lcd.print(" cm");
      return true;
    }
  }
  
  return false;
}

#endif // FONCTIONS_MOTRICES_H
 // FONCTIONS_MOTRICES_H
