#include "config.h"
#include "hardware.h"
#include "comms.h" // For setLcdText
#include "logger.h" // Include the new logger

// --- HARDWARE OBJECTS DEFINITIONS ---
MX1508 motorA(AIN1, AIN2);
MX1508 motorB(BIN1, BIN2);
Servo Servodirection;
Tourelle tourelle(PINTOURELLE_H, PINTOURELLE_V);
LSM303 *compass;
DFRobot_RGBLCD1602 *lcd;
VL53L1X *vl53;
Adafruit_NeoPixel pixels(NEOPIXEL_COUNT, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
Preferences preferences;


// --- RTOS & INTERRUPT ---
SemaphoreHandle_t robotMutex = NULL;
volatile bool bumperPressed = false;
volatile unsigned long lastBumperPressTime = 0;

void IRAM_ATTR onBumperPress() {
  if (millis() - lastBumperPressTime > BUMPER_DEBOUNCE_DELAY_MS) {
    bumperPressed = true;
    lastBumperPressTime = millis();
  }
}

// --- ULTRASONIC INTERRUPT ---
volatile unsigned long echo_start_time = 0;
volatile unsigned long echo_end_time = 0;
volatile bool echo_received = false;
volatile bool is_measuring = false;

void IRAM_ATTR onEcho() {
  if (digitalRead(ECHO) == HIGH) {
    echo_start_time = micros();
    is_measuring = true;
  } else {
    if(is_measuring) {
      echo_end_time = micros();
      echo_received = true;
      is_measuring = false;
    }
  }
}


// --- Tourelle Class Implementation ---
Tourelle::Tourelle(int pinH, int pinV) : servoH_pin(pinH), servoV_pin(pinV) {}

void Tourelle::attach() {
    servoH.attach(servoH_pin);
    servoV.attach(servoV_pin);
}

void Tourelle::detach() {
    servoH.detach();
    servoV.detach();
}

void Tourelle::write(int angleH, int angleV) {
    servoH.write(angleH);
    servoV.write(angleV);
}

int Tourelle::getAngleHorizontal() {
    return servoH.read();
}

int Tourelle::getAngleVertical() {
    return servoV.read();
}


// --- GENERAL ---
void Arret() {
    motorA.motorBrake();
    motorB.motorBrake();
}

void updateBatteryStatus(Robot& robot) {
  static unsigned long low_battery_start_time = 0;
  static bool is_critical_pending = false;
  
  int batteryLevel = readBatteryPercentage();

  // --- Critical Battery Check with Hysteresis ---
  if (batteryLevel < CRITICAL_BATTERY_LEVEL) {
    if (!is_critical_pending && !robot.batteryIsCritical) {
      is_critical_pending = true;
      low_battery_start_time = millis();
    } else if (is_critical_pending && !robot.batteryIsCritical) {
      if (millis() - low_battery_start_time > CRITICAL_BATTERY_HYSTERESIS_MS) {
        robot.batteryIsCritical = true;
        robot.batteryIsLow = true;
        is_critical_pending = false;
      }
    }
    return;
  } 
  else {
    if (is_critical_pending) {
      is_critical_pending = false;
    }
  }

  if (robot.batteryIsLow || robot.batteryIsCritical) {
    if (batteryLevel >= RECHARGED_THRESHOLD) {
      LOG_INFO("Batterie OK. Reprise des operations.");
      robot.speedAvg = robot.initialSpeedAvg;
      robot.speedSlow = robot.initialSpeedSlow;
      robot.batteryIsLow = false;
      robot.batteryIsCritical = false;
    }
  } 
  else {
    if (batteryLevel < LOW_BATTERY_THRESHOLD) {
      if (!robot.batteryIsLow) {
        LOG_INFO("Batterie faible. Vitesse reduite.");
        robot.speedAvg = robot.speedSlow; 
        robot.speedSlow = robot.speedSlow / 2; 
        robot.batteryIsLow = true;
      }
    }
  }
}



// --- Pathfinding functions ---

// NEW: Centralized scanning function
void scanDistances(Robot& robot) {
    LOG_DEBUG("Starting full environment scan...");
    for (int angle = SCAN_H_START_ANGLE; angle <= SCAN_H_END_ANGLE; angle += SCAN_H_STEP) {
        tourelle.write(angle, robot.servoNeutralTurret);
        delay(robot.turretScanDelay > 0 ? robot.turretScanDelay : 30); // Use a default delay if not set

        int distLaser = robot.maxUltrasonicDistance; // Default to max distance
        if (robot.laserInitialized) {
            distLaser = vl53->readRangeContinuousMillimeters() / MM_TO_CM_DIVISOR;
            if (vl53->timeoutOccurred() || distLaser <= 0) {
                distLaser = robot.maxUltrasonicDistance;
            }
        }
        robot.scanDistances[angle] = distLaser;
    }
    tourelle.write(robot.servoNeutralTurret, robot.servoNeutralTurret);
    LOG_DEBUG("Scan complete.");
}

// REFACTORED to use scanDistances
int findClearestPath(Robot& robot) {
    scanDistances(robot); // Perform the scan first

    int bestAngle = -1;
    float maxScore = -9999.0f;

    for (int angle = SCAN_H_START_ANGLE; angle <= SCAN_H_END_ANGLE; angle += SCAN_H_STEP) {
        int effectiveDist = robot.scanDistances[angle];
        float angleDeviation = abs(90 - angle);
        float score = effectiveDist - (angleDeviation * robot.anglePenaltyFactor);

        if (effectiveDist > robot.minDistForValidPath && score > maxScore) {
            maxScore = score;
            bestAngle = angle;
        }
    }
    LOG_DEBUG("findClearestPath result: angle=%d, score=%.2f", bestAngle, maxScore);
    return bestAngle;
}

// NEW: Advanced pathfinding algorithm
int findWidestPath(Robot& robot) {
    scanDistances(robot); // Use the centralized scanner

    int best_start_angle = -1;
    int max_len = 0;
    int current_start_angle = -1;
    int current_len = 0;

    for (int angle = SCAN_H_START_ANGLE; angle <= SCAN_H_END_ANGLE; angle += SCAN_H_STEP) {
        if (robot.scanDistances[angle] > robot.minDistForValidPath) {
            if (current_start_angle == -1) {
                current_start_angle = angle;
            }
            current_len++;
        } else {
            if (current_len > max_len) {
                max_len = current_len;
                best_start_angle = current_start_angle;
            }
            current_start_angle = -1;
            current_len = 0;
        }
    }
    if (current_len > max_len) {
        max_len = current_len;
        best_start_angle = current_start_angle;
    }

    if (best_start_angle != -1) {
        int middleAngle = best_start_angle + (max_len / 2) * SCAN_H_STEP;
        LOG_DEBUG("findWidestPath result: widest segment of %d steps found at %d deg. Middle: %d", max_len, best_start_angle, middleAngle);
        return middleAngle;
    }

    LOG_DEBUG("findWidestPath: No valid path found.");
    return -1;
}
