#ifndef SUPPORT_H
#define SUPPORT_H

// --- Forward Declarations for Command Handlers ---
void handleGlobalCommands(const String& serialordre);
void handleManualModeCommands(const String& serialordre);
void handleAutoModeCommands(const String& serialordre);
void handleParameterCommands(const String& serialordre);
void handleCompassCommands(const String& serialordre);
void handleGeneralCommands(const String& serialordre);
void sendPeriodicData();

void Batterie() {
  int Vbat = analogRead(PINBAT);
  int percentage = map(Vbat, BATTERY_CRITICAL_THRESHOLD, 1023, 0, 100);
  percentage = constrain(percentage, 0, 100);
  Serial.print("Batterie: ");
  Serial.print(percentage);
  Serial.println("%\n");

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Vbat: ");
  lcd.print(Vbat);
  
  if (Vbat < BATTERY_THRESHOLD) {
    Serial.println("ATTENTION: Batterie faible!");
    lcd.clear();
    lcd.print("Batterie faible!");
    lcd.setCursor(0,1);
    lcd.print("Chargez la batterie");
    
    // Set a flag instead of blocking - let the main loop handle this
    // The robot will continue to function but with warnings
    return;
  }
}

float calculateHeading(LSM303 &compass) {
  float heading = (atan2(compass.m.y, compass.m.x) * 180) / PI;
  if (heading < 0) {
    heading += 360;  // Adjust angle to be between 0 and 360
  }
  
  // Correction for inverted compass if necessary
  if (compassInverted) {
    heading += 180.0;
    if (heading >= 360.0) {
      heading -= 360.0;
    }
  }
  
  return heading;
}

void Mcap(int j) {
  static int sampleCount = 0;
  static float headingSum = 0;
  static unsigned long lastSampleTime = 0;

  if (sampleCount == 0) {
    // Start of a new measurement
    headingSum = 0;
    lastSampleTime = millis();
  }

  if (millis() - lastSampleTime > 50) { // Take a sample every 50ms
    compass.read();
    headingSum += calculateHeading(compass);
    sampleCount++;
    lastSampleTime = millis();
  }

  if (sampleCount >= j) {
    cap = headingSum / j;
    sampleCount = 0; // Reset for next time

    Serial.print("Cap: ");
    Serial.print((int)cap);
    Serial.print(", Cible: ");
    Serial.println((int)Ncap);
  }
}

void PhareAllume(){
  digitalWrite(PINPHARE,HIGH);
  Serial.println("Phare allume"); 
}

void Phareeteint(){
  digitalWrite(PINPHARE,LOW);
   Serial.println("Phare eteint");
}

// Function to adjust the heading without exceeding 360°
void ajusterCap(int angle) {
  // Use modulo arithmetic to prevent overflow and ensure proper wrapping
  Ncap = ((Ncap + angle) % 360 + 360) % 360;
}

// Helper function to validate numeric input
bool isValidNumericInput(const String& input, int minVal, int maxVal) {
  if (input.length() == 0) return false;
  
  // Check if all characters are digits (or negative sign at start)
  for (int i = 0; i < input.length(); i++) {
    char c = input.charAt(i);
    if (i == 0 && c == '-') continue; // Allow negative sign at start
    if (c < '0' || c > '9') return false;
  }
  
  int value = input.toInt();
  return (value >= minVal && value <= maxVal);
}

void Terminal() {
  if (Serial.available()) {
    String serialordre = Serial.readStringUntil('\n');
    serialordre.trim();

    handleGlobalCommands(serialordre);
  }
}

void handleGlobalCommands(const String& serialordre) {
    if (serialordre == "stop") {
      Arret();
      changeState(IDLE);
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Stop");
      return;
    } else if (serialordre == "manual") {
      if (DEBUG_MODE) Serial.println("Mode de navigation : MANUEL");
      lcd.clear(); 
      lcd.print("Mode: Manuel");
      currentNavMode = MANUAL_CONTROL;
      Arret(); // Stop the robot when changing mode
      changeState(IDLE);
      return;
    } else if (serialordre == "auto") {
      if (DEBUG_MODE) Serial.println("Mode de navigation : CAP (AUTO)");
      lcd.clear(); 
      lcd.print("Mode: Auto (Cap)");
      currentNavMode = HEADING_CONTROL;
      // Initialize the target heading to the current heading
      compass.read();
      float capActuel = getCalibratedHeading(compass);
      Ncap = capActuel;
      changeState(FOLLOW_HEADING);
      vitesseCible = 0; // The robot waits for the order to move forward
      lcd.setCursor(0,1);
      lcd.print("Cap: " + String((int)Ncap) + "°");
      return;
    }

    handleParameterCommands(serialordre);
}

void handleParameterCommands(const String& serialordre) {
    if (serialordre.startsWith("cap")) {
      String NcapStr = serialordre.substring(serialordre.indexOf('=') + 1);
      
      if (isValidNumericInput(NcapStr, 0, 359)) {
        Ncap = NcapStr.toInt();
        if (DEBUG_MODE) {
          Serial.print("Cap regle a : ");
          Serial.println(Ncap);
        }
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Cap : " + String(Ncap));
        changeState(FOLLOW_HEADING);
      } else {
        if (DEBUG_MODE) Serial.println("Commande invalide pour cap (0-359)");
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Cap invalide");
        lcd.setCursor(0,1);
        lcd.print("Utilisez 0-359");
      }
    } else if(serialordre.startsWith("vitesse")){
      String VitesseStr = serialordre.substring(7);
      
      if (isValidNumericInput(VitesseStr, 0, 255)) {
        int vitesseP = VitesseStr.toInt();
        vitesseCible = vitesseP;
        if (DEBUG_MODE) Serial.println("Vitesse cible : " + String(vitesseCible));
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Vitesse : ");
        lcd.print(vitesseCible);
      } else {
        if (DEBUG_MODE) Serial.println("Erreur : Vitesse invalide (0-255)");
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Vitesse invalide");
        lcd.setCursor(0,1);
        lcd.print("Utilisez 0-255");
      }
    } else if (serialordre.startsWith("servo")) {
      String servoStr = serialordre.substring(5);
      
      if (isValidNumericInput(servoStr, 0, 180)) {
        int servo = servoStr.toInt();
        if (DEBUG_MODE) Serial.println("Servo : " + String(servo));
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Servo : ");
        lcd.print(servo);
        Servodirection.write(servo);
      } else {
        if (DEBUG_MODE) Serial.println("Erreur : Angle servo invalide (0-180)");
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Servo invalide");
        lcd.setCursor(0,1);
        lcd.print("Utilisez 0-180");
      }
    } else if(serialordre.startsWith("distance")){
      String distanceStr = serialordre.substring(8);
      
      if (isValidNumericInput(distanceStr, MANUAL_MIN_DISTANCE, MANUAL_MAX_DISTANCE)) {
        int newDistance = distanceStr.toInt();
        setManualDistance(newDistance);
        if (DEBUG_MODE) {
          Serial.print("Distance manuelle reglee a:");
          Serial.print(newDistance);
          Serial.println(" cm");
        }
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Distance: ");
        lcd.print(newDistance);
        lcd.print(" cm");
      } else {
        if (DEBUG_MODE) {
          Serial.print("Erreur: Distance invalide (");
          Serial.print(MANUAL_MIN_DISTANCE);
          Serial.print("-");
          Serial.print(MANUAL_MAX_DISTANCE);
          Serial.println(" cm)");
        }
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Distance invalide");
        lcd.setCursor(0,1);
        lcd.print("Utilisez 5-100 cm");
      }
    } else if(serialordre.startsWith("virageprecis")){
      // Format: virageprecis90 or virageprecis-45 (angle in degrees)
      String angleStr = serialordre.substring(12);
      
      // Validate angle input (-180 to 180 degrees)
      if (isValidNumericInput(angleStr, -180, 180)) {
        float angle = angleStr.toFloat();
        
        if (angle != 0) {
          if (DEBUG_MODE) {
            Serial.print("INITIATION: Virage PRECIS de ");
            Serial.print(angle);
            Serial.println(" degres");
          }
          
          compass.read();
          float capActuel = getCalibratedHeading(compass);
          capCibleRotation = calculateTargetHeading(capActuel, angle);
          
          // Use the precise turn function
          executePreciseTurn(capCibleRotation);
        } else {
          if (DEBUG_MODE) Serial.println("Erreur : Angle de virage precis ne peut pas etre 0");
          lcd.clear();
          lcd.setCursor(0,0);
          lcd.print("Angle invalide");
          lcd.setCursor(0,1);
          lcd.print("Utilisez -180 a 180");
        }
      } else {
        if (DEBUG_MODE) Serial.println("Erreur : Angle de virage precis invalide (-180 a 180)");
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Virage precis invalide");
        lcd.setCursor(0,1);
        lcd.print("Utilisez -180 a 180");
      }
    } else if(serialordre.startsWith("virage")){
      // Format: virage90 or virage-45 (angle in degrees) - standard turn
      String angleStr = serialordre.substring(6);
      
      // Validate angle input (-180 to 180 degrees)
      if (isValidNumericInput(angleStr, -180, 180)) {
        float angle = angleStr.toFloat();
        
        if (angle != 0) {
          if (DEBUG_MODE) {
            Serial.print("INITIATION: Virage STANDARD de ");
            Serial.print(angle);
            Serial.println(" degres");
          }
          
          compass.read();
          float capActuel = getCalibratedHeading(compass);
          capCibleRotation = calculateTargetHeading(capActuel, angle);
          vitesseCible = VITESSE_LENTE;
          
          if (angle > 0) {
            changeState(TURNING_RIGHT);
            lcd.clear();
            lcd.setCursor(0,0);
            lcd.print("Droite ");
            lcd.print((int)angle);
            lcd.print(" deg");
          } else {
            changeState(TURNING_LEFT);
            lcd.clear();
            lcd.setCursor(0,0);
            lcd.print("Gauche ");
            lcd.print((int)abs(angle));
            lcd.print(" deg");
          }
          lcd.setCursor(0,1);
          lcd.print("Cap: ");
          lcd.print((int)capCibleRotation);
          lcd.print(" deg");
        } else {
          if (DEBUG_MODE) Serial.println("Erreur : Angle de virage ne peut pas etre 0");
          lcd.clear();
          lcd.setCursor(0,0);
          lcd.print("Angle invalide");
          lcd.setCursor(0,1);
          lcd.print("Utilisez -180 a 180");
        }
      } else {
        if (DEBUG_MODE) Serial.println("Erreur : Angle de virage invalide (-180 a 180)");
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Virage invalide");
        lcd.setCursor(0,1);
        lcd.print("Utilisez -180 a 180");
      }
    }

    handleCompassCommands(serialordre);
}

void handleCompassCommands(const String& serialordre) {
    if(serialordre=="capactuel"){
      compass.read();
      float capActuel = getCalibratedHeading(compass);
      Serial.print("Cap actuel: ");
      Serial.println(capActuel);
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Cap: ");
      lcd.print((int)capActuel);
      lcd.print(" deg");
    } else if(serialordre=="calibrer"){
      changeState(CALIBRATING_COMPASS); // Change state to start calibration
    } else if(serialordre=="debugcompas"){
      debugCompass();
    } else if(serialordre=="compasinfo"){
      displayCompassInfo();
    } else if(serialordre=="resetcalib"){
      compassOffset = 0.0;
      compassCalibrated = false;
      saveCompassCalibration(); // Persist the reset
      if (DEBUG_MODE) Serial.println("Calibration du compas réinitialisée");
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Calib reset");
    }

    handleGeneralCommands(serialordre);
}

void handleGeneralCommands(const String& serialordre) {
    if (serialordre == "off") {
      Phareeteint();
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Phares off");
    } else if (serialordre == "on") {
      PhareAllume();
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Phares on");
    } else if (serialordre == "dusm") {
      // The 'dusm' variable is continuously updated by the sensor_task
      Serial.print("Distance US: ");
      Serial.print(dusm);
      Serial.println(" cm");
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Distance : ");
      lcd.print(dusm);
    } else if (serialordre == "obstacle") {
      if (DEBUG_MODE) Serial.println("Mode : obstacle");
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Mode : ");
      lcd.print("Obstacle");
      vitesseCible = VITESSE_LENTE;
      changeState(OBSTACLE_AVOIDANCE);
    } else if (serialordre == "detect" || serialordre == "pir") {
      Phareeteint();
      if (DEBUG_MODE) Serial.println("Mode : detection");
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Ne pas bouger");
      changeState(PIR_DETECT);
    } else if(serialordre=="Vbat"){
      Batterie();
    } else if(serialordre=="mode"){
      if (DEBUG_MODE) Serial.print("Mode de navigation actuel: ");
      if (currentNavMode == MANUAL_CONTROL) {
        if (DEBUG_MODE) Serial.println("MANUEL");
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Mode: Manuel");
        lcd.setCursor(0,1);
        lcd.print("Controle direct");
      } else {
        if (DEBUG_MODE) Serial.println("CAP (AUTO)");
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Mode: Auto");
        lcd.setCursor(0,1);
        lcd.print("Cap: ");
        lcd.print((int)Ncap);
        lcd.print(" deg");
      }
    }

    if (currentNavMode == MANUAL_CONTROL) {
        handleManualModeCommands(serialordre);
    } else if (currentNavMode == HEADING_CONTROL) {
        handleAutoModeCommands(serialordre);
    }
}

void handleManualModeCommands(const String& serialordre) {
    if (serialordre == "U") {
      if (DEBUG_MODE) {
        Serial.print("Avance de ");
        Serial.print(manualDistance);
        Serial.println(" cm (manuel)");
      }
      executeManualDistanceMove(true); // true = forward
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Avance ");
      lcd.print(manualDistance);
      lcd.print(" cm");
    } else if (serialordre == "D") {
      if (DEBUG_MODE) Serial.println("Recule (manuel)");
      vitesseCible = VITESSE_LENTE;
      changeState(MANUAL_MOVING_BACKWARD);
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Recule");
      lcd.setCursor(0,1);
      lcd.print("Mode manuel");
    } else if (serialordre == "L") {
      if (DEBUG_MODE) Serial.println("Tourne a Gauche (manuel)");
      vitesseCible = VITESSE_LENTE;
      changeState(MANUAL_TURNING_LEFT);
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Gauche");
      lcd.setCursor(0,1);
      lcd.print("Mode manuel");
    } else if (serialordre == "R") {
      if (DEBUG_MODE) Serial.println("Tourne a Droite (manuel)");
      vitesseCible = VITESSE_LENTE;
      changeState(MANUAL_TURNING_RIGHT);
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Droite");
      lcd.setCursor(0,1);
      lcd.print("Mode manuel");
    }
}

void handleAutoModeCommands(const String& serialordre) {
    if (serialordre == "U") {
      if (DEBUG_MODE) Serial.println("Avance en suivant le cap");
      changeState(FOLLOW_HEADING);
      vitesseCible = VITESSE_MOYENNE; // Give the order to move forward
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Auto: Avance");
      lcd.setCursor(0,1);
      lcd.print("Cap: ");
      lcd.print((int)Ncap);
      lcd.print(" deg");
    } else if (serialordre == "D") {
      if (DEBUG_MODE) Serial.println("Arret avec maintien du cap");
      changeState(MAINTAIN_HEADING);
      vitesseCible = 0; // Full stop
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Auto: Arret");
      lcd.setCursor(0,1);
      lcd.print("Cap maintient: ");
      lcd.print((int)Ncap);
      lcd.print(" deg");
    } else if (serialordre == "L") {
      ajusterCap(-15); // Modify the target heading by -15 degrees
      if (DEBUG_MODE) {
        Serial.print("Nouveau cap cible : "); Serial.println(Ncap);
      }
      
      // If we were in MAINTAIN_HEADING, we remain in MAINTAIN_HEADING
      // If we were in FOLLOW_HEADING, we remain in FOLLOW_HEADING
      if (currentState == MAINTAIN_HEADING) {
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Auto: Gauche");
        lcd.setCursor(0,1);
        lcd.print("Cap maintient: ");
        lcd.print((int)Ncap);
        lcd.print(" deg");
      } else {
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Auto: Gauche");
        lcd.setCursor(0,1);
        lcd.print("Cap: ");
        lcd.print((int)Ncap);
        lcd.print(" deg");
      }
    } else if (serialordre == "R") {
      ajusterCap(15); // Modify the target heading by +15 degrees
      if (DEBUG_MODE) {
        Serial.print("Nouveau cap cible : "); Serial.println(Ncap);
      }
      
      // If we were in MAINTAIN_HEADING, we remain in MAINTAIN_HEADING
      // If we were in FOLLOW_HEADING, we remain in FOLLOW_HEADING
      if (currentState == MAINTAIN_HEADING) {
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Auto: Droite");
        lcd.setCursor(0,1);
        lcd.print("Cap maintient: ");
        lcd.print((int)Ncap);
        lcd.print(" deg");
      } else {
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Auto: Droite");
        lcd.setCursor(0,1);
        lcd.print("Cap: ");
        lcd.print((int)Ncap);
        lcd.print(" deg");
      }
    }
}

void sendPeriodicData() {
    static unsigned long lastSendTime = 0;
    unsigned long currentTime = millis();

    if (currentTime - lastSendTime >= 500) { // Send data every 500ms
        lastSendTime = currentTime;

        // Send speed
        Serial.print("Vitesse: ");
        Serial.println(vitesseCourante);

        // Send LCD text
        // This is tricky because we don't know the current LCD content.
        // A better approach would be to have a global variable for the LCD content.
        // For now, I'll just send a placeholder.
        Serial.println("LCD: Placeholder");

        // Send PIR status
        if (digitalRead(PIR) == LOW) {
            Serial.println("PIR: mouvement détecté");
        } else {
            Serial.println("PIR: aucun mouvement");
        }
    }
}

#endif // SUPPORT_H
