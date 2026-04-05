#include "config.h"
#include "hardware.h"
#include "robot.h"
#include "battery_utils.h"
#include "comms.h" // For setLcdText
#include "logger.h" // Include the new logger
#include <EEPROM.h>
#include <SPI.h>


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

// LCD availability flag - set to false by default, only true if LCD responds on I2C
bool lcdAvailable = false;


// --- RTOS & INTERRUPT ---
SemaphoreHandle_t robotMutex = NULL;
volatile bool bumperPressed = false;
volatile uint32_t lastBumperPressTime = 0;

void IRAM_ATTR onBumperPress() {
  // Use FreeRTOS tick count instead of millis() in ISR
  // millis() is not safe to call from ISR on some platforms
  uint32_t currentTicks = xTaskGetTickCountFromISR();
  uint32_t debounceDelayTicks = pdMS_TO_TICKS(BUMPER_DEBOUNCE_DELAY_MS);
  
  if ((currentTicks - lastBumperPressTime) > debounceDelayTicks) {
    bumperPressed = true;
    lastBumperPressTime = currentTicks;
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
    attached = true;
    LOG_DEBUG("Tourelle servos attached (pins %d, %d)", servoH_pin, servoV_pin);
}

void Tourelle::detach() {
    servoH.detach();
    servoV.detach();
    attached = false;
    LOG_DEBUG("Tourelle servos detached");
}

void Tourelle::write(int angleH, int angleV) {
    if (!attached) {
        LOG_WARN("Tourelle not attached, skipping write command");
        return;
    }
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
    for (int angle = SCAN_H_START_ANGLE; angle <= SCAN_H_END_ANGLE; angle += SCAN_H_STEP) {
        tourelle.write(angle, robot.servoNeutralTurret);
        delay(robot.turretScanDelay > 0 ? robot.turretScanDelay : 30); // Use a default delay if not set

        int distLaser = robot.maxUltrasonicDistance; // Default to max distance
        if (robot.laserInitialized && vl53->dataReady()) {
            distLaser = vl53->readRangeContinuousMillimeters() / MM_TO_CM_DIVISOR;
            if (vl53->timeoutOccurred() || distLaser <= 0) {
                distLaser = robot.maxUltrasonicDistance;
            }
        } else if (robot.laserInitialized) {
            // Data not ready, use safe default instead of potentially uninitialized value
            distLaser = robot.maxUltrasonicDistance;
        }
        robot.scanDistances[angle] = distLaser;
    }
    tourelle.write(robot.servoNeutralTurret, robot.servoNeutralTurret);
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
    LOG_DEBUG("findClearestPath result: angle=%d", bestAngle);
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
        LOG_DEBUG("findWidestPath: found path at %d deg", middleAngle);
        return middleAngle;
    }

    return -1;
}

// Function to clear a stuck I2C bus.
// To be called before Wire.begin()
void clearI2CBus() {
  // Set pins to INPUT_PULLUP to allow pull-up resistors to work
  pinMode(SDA_PIN, INPUT_PULLUP);
  pinMode(SCL_PIN, INPUT_PULLUP);
  delay(10); // Allow pins to settle

  // Check if bus is already clear
  if (digitalRead(SDA_PIN) == 1 && digitalRead(SCL_PIN) == 1) {
    LOG_DEBUG("I2C bus is clear.");
    return;
  }

  LOG_INFO("I2C bus is stuck. Attempting to clear...");

  // Toggle SCL to try to unstick SDA
  pinMode(SCL_PIN, OUTPUT);
  digitalWrite(SCL_PIN, LOW);
  delayMicroseconds(10);
  
  for (int i = 0; i < 9 && digitalRead(SDA_PIN) == 0; i++) {
    // Toggle SCL clock 9 times (standard I2C recovery)
    digitalWrite(SCL_PIN, HIGH);
    delayMicroseconds(5);
    digitalWrite(SCL_PIN, LOW);
    delayMicroseconds(5);
  }

  // Generate STOP condition
  digitalWrite(SCL_PIN, HIGH);
  delayMicroseconds(5);
  
  pinMode(SDA_PIN, INPUT_PULLUP);
  pinMode(SCL_PIN, INPUT_PULLUP);
  delay(10);

  if (digitalRead(SDA_PIN) == 0) {
    LOG_ERROR("Failed to clear I2C bus. SDA is still stuck. Check hardware for short circuit.");
  } else {
    LOG_INFO("I2C bus unstuck successfully.");
  }
}

// Function to scan the I2C bus for connected devices WITH TIMEOUT PROTECTION
void scanI2CBus() {
  LOG_DEBUG("Scanning I2C bus (with timeout protection)...");
  
  // Set I2C timeout to prevent hanging on unresponsive devices
  // ESP32 Wire library timeout: default is very large, limit it to 50ms
  Wire.setTimeOut(50);  // 50ms timeout
  
  byte error, address;
  int nDevices = 0;
  
  for(address = 1; address < 127; address++ ) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission(true);  // true = send STOP condition
    
    if (error == 0) {
      LOG_DEBUG("I2C device found at address 0x%02x", address);
      nDevices++;
    } else if (error == 4) {
      LOG_DEBUG("Unknown error at address 0x%02x", address);
    }
    // Other error codes: 1=data too long, 2=address NACK, 3=data NACK, 4=other
    
    delay(5); // Small delay between probes
  }
  
  if (nDevices == 0) {
    LOG_WARN("No I2C devices found.");
  } else {
    LOG_DEBUG("I2C scan complete: found %d devices.", nDevices);
  }
}

// Headlight control
void headlightOn() {
  digitalWrite(PIN_PHARE, HIGH);
}

void headlightOff() {
  digitalWrite(PIN_PHARE, LOW);
}

// Initialize LCD without calling Wire.begin() (which blocks if device doesn't respond)
// This is a workaround for DFRobot_RGBLCD1602::init() calling Wire.begin()
void initializeLCD_Safe() {
  if (!lcd) {
    LOG_ERROR("LCD object is NULL!");
    return;
  }
  
  // DFRobot init() would call Wire.begin() here which blocks
  // Instead, we manually do what init() does but skip Wire.begin()
  LOG_INFO("Initializing LCD display...");
  
  // Call the begin() method directly (inherited from LiquidCrystal)
  // This sets up the LCD communication but won't re-init Wire
  // We need to use reflection or call it through the LCD object
  
  // For DFRobot at 0x60, these are the color registers
  if (LCD_I2C_ADDR == 0x60) {
    // Perform I2C ACK probe to verify LCD is present
    LOG_INFO("Probing LCD at 0x%02x...", LCD_I2C_ADDR);
    Wire.beginTransmission(LCD_I2C_ADDR);
    byte error = Wire.endTransmission(true);  // Send STOP condition
    
    if (error == 0) {
      // Device responded, now call init()
      LOG_INFO("LCD found at 0x%02x - calling init()", LCD_I2C_ADDR);
      lcd->init();
      lcd->setBacklight(true);
      LOG_INFO("LCD initialized successfully");
      lcdAvailable = true;
    } else {
      LOG_ERROR("LCD NOT responding at 0x%02x (I2C error=%d)", LCD_I2C_ADDR, error);
      lcdAvailable = false;
    }
  } else {
    LOG_WARN("LCD at unusual address 0x%02x, may not work", LCD_I2C_ADDR);
    lcdAvailable = false;
  }
}