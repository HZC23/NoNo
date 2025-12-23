#ifndef SD_UTILS_H
#define SD_UTILS_H

#include <Arduino.h>
#include <SD.h>
#include "state.h" // Include for Robot struct

bool setupSDCard();
void getRandomJokeFromSD(Robot& robot, const char* filename, char* buffer, size_t bufferSize);
bool loadConfig(Robot& robot);
#if USB_MSC_ENABLED
void startUSBMassStorage(Robot& robot);
void stopUSBMassStorage(Robot& robot);
#endif

#endif // SD_UTILS_H
