#ifndef HARDWARE_H
#define HARDWARE_H

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SdFat.h>
#include <FS_MX1508.h>
#include <Esp32Servo.h>
#include <LSM303.h>
#include <VL53L1X.h>
#include <DFRobot_RGBLCD1602.h>
#include <Adafruit_NeoPixel.h>
#include <Preferences.h>

// Forward Declaration
struct Robot;

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
extern volatile unsigned long lastBumperPressTime;
void IRAM_ATTR onBumperPress();

// --- ULTRASONIC INTERRUPT ---
extern volatile unsigned long echo_start_time;
extern volatile unsigned long echo_end_time;
extern volatile bool echo_received;
extern volatile bool is_measuring;
void IRAM_ATTR onEcho();


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
extern SdFat sd;


// --- HARDWARE-RELATED FUNCTION PROTOTYPES ---

// General
void clearI2CBus();
void scanI2CBus();
void Arret();
void updateBatteryStatus(Robot& robot);
void scanDistances(Robot& robot); // Added
int findClearestPath(Robot& robot); // Added
int findWidestPath(Robot& robot); // Added

// Compass
bool isNVSDataValid();
void saveCompassCalibration(const Robot& robot);
void loadCompassCalibration(Robot& robot);
void compass_init(Robot& robot);
float calculateHeading(float y, float x);
float calculateHeading(const LSM303& compass);
float getCalibratedHeading(Robot& robot);
void calibrateCompass(Robot& robot);
float getPitch(Robot& robot);
bool detectImpactOrStall(Robot& robot);

// LED FX
void led_fx_init();
void led_fx_update(const Robot& robot);
void led_fx_set_all(uint8_t r, uint8_t g, uint8_t b);
void led_fx_off();

// SD Utils
bool setupSDCard();
void getRandomJokeFromSD(Robot& robot, const char* filename, char* buffer, size_t bufferSize);
bool loadConfig(Robot& robot);
SdFat* get_sd_card();

// Sensor Task
void sensor_init();
void sensor_update_task(Robot& robot);

// Headlight Control
void headlightOn();
void headlightOff();

// Turret Control
void updateTurret(Robot& robot, bool isMovingForward);
void syncTurretWithSteering(int steeringAngle);
int setAckermannAngle(Robot& robot, int angleError, int speed);


#endif // HARDWARE_H