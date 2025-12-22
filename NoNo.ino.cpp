# 1 "C:\\Users\\Hadrien\\AppData\\Local\\Temp\\tmpo17pda89"
#include <Arduino.h>
# 1 "C:/Users/Hadrien/Documents/PlatformIO/Projects/Nono/src/NoNo.ino"

#include <Arduino.h>

#include <ArduinoJson.h>
#include <Preferences.h>

#include "config.h"
#include "state.h"
#include "compass.h"
#include "display.h"
#include "terminal.h"
#include "sensor_task.h"
#include "fonctions_motrices.h"
#include "led_fx.h"
#include "support.h"
#include "telemetry.h"
#include "sd_utils.h"
#include "turret_control.h"


#include "xbox_controller_bluepad.h"



SemaphoreHandle_t robotMutex = NULL;


volatile bool bumperPressed = false;


MX1508 motorA(AIN1, AIN2);
MX1508 motorB(BIN1, BIN2);
Servo Servodirection;
Tourelle tourelle(PINTOURELLE_H, PINTOURELLE_V);
LSM303 *compass;
DFRobot_RGBLCD1602 *lcd;
VL53L1X *vl53;


Robot robot;
unsigned long startTime;
Preferences preferences;
XboxControllerBluepad xboxController(robot);
unsigned long lastCompassReadTime = 0;
void IRAM_ATTR onBumperPress();
void setup();
void loop();
#line 47 "C:/Users/Hadrien/Documents/PlatformIO/Projects/Nono/src/NoNo.ino"
void IRAM_ATTR onBumperPress() {
  bumperPressed = true;
}



void updateBatteryStatus(Robot& robot);

void setup() {
    Serial.begin(SERIAL_BAUD_RATE);
    if (DEBUG_MODE) Serial.println("--- NONO BOOTING ---");
    startTime = millis();


    bool nvsOpened = preferences.begin(NVS_NAMESPACE, true);
    if (!nvsOpened) {

        nvsOpened = preferences.begin(NVS_NAMESPACE, false);
        if (nvsOpened) {

            preferences.putInt(NVS_COMM_MODE_KEY, COMM_MODE_BLE_APP);
            if (DEBUG_MODE) Serial.println("NVS namespace created and default comm mode set.");
        } else {

            if (DEBUG_MODE) Serial.println("CRITICAL ERROR: Failed to open NVS namespace. Proceeding with default.");


        }
    }



    robot.activeCommMode = (CommunicationMode)preferences.getInt(NVS_COMM_MODE_KEY, COMM_MODE_BLE_APP);
    preferences.end();


    robotMutex = xSemaphoreCreateMutex();

    if (!Wire.begin(SDA_PIN, SCL_PIN)) {
        if (DEBUG_MODE) Serial.println("Erreur d'initialisation I2C");
    }
    Wire.setClock(400000);
    delay(500);


    compass = new LSM303();
    lcd = new DFRobot_RGBLCD1602(LCD_I2C_ADDR, LCD_LINE_LENGTH, LCD_ROWS);
    vl53 = new VL53L1X();


    if (!setupSDCard()) {
      setLcdText(robot, "SD Card Error!");
      while (true);
    }
    delay(500);
    lcd->init();


    if (robot.activeCommMode == COMM_MODE_XBOX) {
        if (DEBUG_MODE) Serial.println("COMM MODE: Xbox Controller");
        xboxController.begin();
    } else {
        if (DEBUG_MODE) Serial.println("COMM MODE: Serial Only");
    }

    setLcdText(robot, LCD_STARTUP_MESSAGE_1);


    pinMode(PIR, INPUT);
    pinMode(VBAT, INPUT);
    pinMode(INTERUPTPIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(INTERUPTPIN), onBumperPress, FALLING);
    pinMode(PIN_PHARE, OUTPUT);

    led_fx_init();


    sensor_init();
    compass_init(robot);
    robot.compassInverted = COMPASS_IS_INVERTED;


    robot.speedAvg = VITESSE_MOYENNE;
    robot.speedSlow = VITESSE_LENTE;

    Servodirection.attach(PINDIRECTION);
    Servodirection.write(NEUTRE_DIRECTION);
    tourelle.attach();
    tourelle.write(SCAN_CENTER_ANGLE, NEUTRE_TOURELLE);

    if (robot.activeCommMode == COMM_MODE_XBOX) {
      xboxController.setHardware(motorA, motorB, tourelle);
    }

    if (DEBUG_MODE) Serial.println("--- SETUP COMPLETE ---");
    setLcdText(robot, LCD_STARTUP_MESSAGE_2);
    delay(SCROLL_DELAY_MS);
    digitalWrite(PIN_PHARE, LOW);
}




void loop() {

  robot.loopStartTime = millis();

  if(xSemaphoreTake(robotMutex, (TickType_t) MUTEX_WAIT_TICKS) == pdTRUE) {


    if (bumperPressed) {
      bumperPressed = false;
      if (robot.currentState != EMERGENCY_EVASION) {
        changeState(robot, EMERGENCY_EVASION);
      }
    }


    bool isMoving = (robot.currentState == MOVING_FORWARD || robot.currentState == FOLLOW_HEADING || robot.currentState == MAINTAIN_HEADING);
    updateTurret(robot, isMoving);


    if (robot.activeCommMode == COMM_MODE_XBOX) {
      xboxController.processControllers();
    }


    checkSerial();

    updateBatteryStatus(robot);
    led_fx_update(robot);


    sensor_update_task(robot);
    if (robot.currentState != IDLE || (millis() - lastCompassReadTime > COMPASS_READ_INTERVAL_MS)) {
      robot.cap = getCalibratedHeading(robot);
      lastCompassReadTime = millis();
    }
    robot.currentPitch = getPitch(robot);
    if (robot.laserInitialized && vl53->dataReady()) {
      robot.distanceLaser = vl53->readRangeContinuousMillimeters() / MM_TO_CM_DIVISOR;
    }


    displayJokesIfIdle(robot);
    updateLcdDisplay(robot);


    if (!robot.initialActionTaken && robot.currentState == IDLE && millis() - startTime >= INITIAL_AUTONOMOUS_DELAY_MS) {
        changeState(robot, OBSTACLE_AVOIDANCE);
        robot.initialActionTaken = true;
    }


    switch (robot.currentState) {
      case CALIBRATING_COMPASS:
        calibrateCompass(robot);
        break;
      default:
        updateMotorControl(robot);
        break;
    }


    if (millis() - robot.lastReportTime > robot.reportInterval) {
      robot.lastReportTime = millis();
      sendTelemetry(robot);
    }

    xSemaphoreGive(robotMutex);
  }


  robot.loopEndTime = millis();
  unsigned long loopDuration = robot.loopEndTime - robot.loopStartTime;
  if (loopDuration < LOOP_TARGET_PERIOD_MS) {
    delay(LOOP_TARGET_PERIOD_MS - loopDuration);
  }
}


void updateBatteryStatus(Robot& robot) {
  int batteryLevel = readBatteryPercentage();

  if (batteryLevel < CRITICAL_BATTERY_LEVEL) {
    if (!robot.batteryIsCritical) {
      robot.batteryIsCritical = true;
      robot.batteryIsLow = true;
    }
    return;
  }

  if (batteryLevel < LOW_BATTERY_THRESHOLD) {
    if (!robot.batteryIsLow) {
      robot.speedAvg = VITESSE_LENTE;
      robot.speedSlow = VITESSE_LENTE / 2;
      robot.batteryIsLow = true;
    }
  }
  else {
    if (robot.batteryIsLow || robot.batteryIsCritical) {
      robot.speedAvg = VITESSE_MOYENNE;
      robot.speedSlow = VITESSE_LENTE;
      robot.batteryIsLow = false;
      robot.batteryIsCritical = false;
    }
  }
}