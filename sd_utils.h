#ifndef SD_UTILS_H
#define SD_UTILS_H

#include <Arduino.h>
#include <SD.h>
#include "state.h" // Include for Robot struct

bool setupSDCard();
void getRandomJokeFromSD(Robot& robot, const char* filename, char* buffer, size_t bufferSize);
void playMusicFromSD(Robot& robot, const char* filename, int buzzerPin);
void listMusicFiles(Robot& robot, void (*callback)(const char* filename));

#endif // SD_UTILS_H
