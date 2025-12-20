#ifndef HARDWARE_H
#define HARDWARE_H

// This file contains all the hardware-related includes and extern declarations.
// Include this file in any .h or .cpp file that needs to interact with the robot's hardware.

// Hardware Libraries
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <FS_MX1508.h>
#include <Servo.h>
#include <LSM303.h>
#include <VL53L1X.h>
#include <DFRobot_RGBLCD1602.h>
#include <Adafruit_NeoPixel.h>
#include <BluetoothSerial.h>

// Bluepad32 library. It provides its own extern instance "BP32".
#include <Bluepad32.h>


// Hardware Objects (declared as extern)
// The actual objects are defined in NoNo.ino
extern MX1508 motorA;
extern MX1508 motorB;
extern Servo Servodirection;
extern LSM303 compass;
extern DFRobot_RGBLCD1602 lcd;
extern VL53L1X vl53;
extern BluetoothSerial SerialBT;
extern Adafruit_NeoPixel pixels;

// Forward declaration for Tourelle
class Tourelle;
extern Tourelle tourelle;

#endif // HARDWARE_H
