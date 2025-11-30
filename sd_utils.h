#ifndef SD_UTILS_H
#define SD_UTILS_H

#include <Arduino.h>
#include <SD.h> // Include the SD library

// Forward declaration for joke reading function
void getRandomJokeFromSD(const char* filename, char* buffer, size_t bufferSize);
bool setupSDCard();

// Forward declaration for music playback function
void playMusicFromSD(const char* filename, int buzzerPin);
void listMusicFiles(void (*callback)(const char* filename));

#endif // SD_UTILS_H
