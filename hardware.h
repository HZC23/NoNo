#ifndef HARDWARE_H
#define HARDWARE_H

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <FS_MX1508.h>
#include <Esp32Servo.h>
#include <LSM303.h>
#include <VL53L1X.h>
#include <DFRobot_RGBLCD1602.h>
#include <Adafruit_NeoPixel.h>
#include <Preferences.h>
#include "robot.h" // Includes Robot struct and config.h

// --- Tourelle Class Definition ---
class Tourelle {
private:
    Servo servoH;
    Servo servoV;
    int servoH_pin;
    int servoV_pin;

public:
    Tourelle(int pinH, int pinV);
    void attach();
    void detach();
    void write(int angleH, int angleV);
    int getAngleHorizontal();
    int getAngleVertical();
};


// --- RTOS ---
extern SemaphoreHandle_t robotMutex;

// --- BUMPER INTERRUPT ---
extern volatile bool bumperPressed;
void IRAM_ATTR onBumperPress();


// --- HARDWARE OBJECTS (EXTERN) ---
extern MX1508 motorA;
extern MX1508 motorB;
extern Servo Servodirection;
extern Tourelle tourelle;
extern LSM303 *compass;
extern DFRobot_RGBLCD1602 *lcd;
extern VL53L1X *vl53;
extern Adafruit_NeoPixel pixels;
extern Preferences preferences;


// --- HARDWARE-RELATED FUNCTION PROTOTYPES ---

// General
void Arret();
void updateBatteryStatus(Robot& robot);

// Compass (from compass.h)
void compass_init(Robot& robot);
float getCalibratedHeading(Robot& robot);
void calibrateCompass(Robot& robot);
void saveCompassCalibration(const Robot& robot);
void loadCompassCalibration(Robot& robot);
bool isEEPROMDataValid();
float calculateHeading(float y, float x);
float calculateHeading(const LSM303& compass);
float getPitch(Robot& robot);
void displayCompassInfo(Robot& robot);
bool detectImpactOrStall(Robot& robot);
void Mcap(Robot& robot, int n);


// LED FX (from led_fx.h)
void led_fx_init();
void led_fx_update(const Robot& robot);
void led_fx_set_all(uint8_t r, uint8_t g, uint8_t b);
void led_fx_off();

// SD Utils (from sd_utils.h)
bool setupSDCard();
void getRandomJokeFromSD(Robot& robot, const char* filename, char* buffer, size_t bufferSize);
bool loadConfig(Robot& robot);

// Sensor Task (from sensor_task.h)
void sensor_init();
void sensor_update_task(Robot& robot);

// Support (from support.h)
void PhareAllume();
void PhareEteint();
float readBatteryVoltage();
int readBatteryPercentage();

// Turret Control (from turret_control.h)
void updateTurret(Robot& robot, bool isMovingForward);
void syncTurretWithSteering(int steeringAngle);
int findClearestPath(Robot& robot);
int setAckermannAngle(Robot& robot, int angleError, int speed);


#endif // HARDWARE_H