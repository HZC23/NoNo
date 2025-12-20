#include "sd_utils.h"
#include "config.h" // For SD_CS_PIN and DEBUG_MODE
#include "state.h"    // For Robot struct
#include <string.h>

// Helper to check if a string ends with a specific suffix
bool endsWith(const char* str, const char* suffix) {
    if (!str || !suffix) return 0;
    size_t lenstr = strlen(str);
    size_t lensuffix = strlen(suffix);
    if (lensuffix > lenstr) return 0;
    return strncmp(str + lenstr - lensuffix, suffix, lensuffix) == 0;
}

bool setupSDCard() {
    if (DEBUG_MODE) {
        Serial.print(F("Initializing SD card..."));
    }
    if (!SD.begin(SD_CS_PIN)) {
        if (DEBUG_MODE) {
            Serial.println(F("SD Card initialization failed!"));
        }
        return false;
    }
    if (DEBUG_MODE) {
        Serial.println(F("SD Card initialized."));
    }
    return true;
}

void getRandomJokeFromSD(Robot& robot, const char* filename, char* buffer, size_t bufferSize) {
    if (!robot.sdCardReady) {
        if (DEBUG_MODE) {
            Serial.println(F("SD Card not ready for getRandomJokeFromSD."));
        }
        strncpy(buffer, "Error: SD Card not ready!", bufferSize - 1);
        buffer[bufferSize - 1] = '\0';
        return;
    }
    File jokesFile = SD.open(filename);
    if (!jokesFile) {
        if (DEBUG_MODE) {
            Serial.print(F("Error opening "));
            Serial.println(filename);
        }
        strncpy(buffer, "Error: SD file not found!", bufferSize - 1);
        buffer[bufferSize - 1] = '\0';
        return;
    }

    int numLines = 0;
    while (jokesFile.available()) {
        if (jokesFile.read() == '\n') {
            numLines++;
        }
    }
    jokesFile.seek(0);

    if (numLines == 0) {
        jokesFile.close();
        strncpy(buffer, "Error: No jokes found!", bufferSize - 1);
        buffer[bufferSize - 1] = '\0';
        return;
    }

    long randomIndex = random(numLines);

    for (long i = 0; i < randomIndex; i++) {
        while(jokesFile.available() && jokesFile.read() != '\n');
    }

    if (jokesFile.available()) {
        size_t bytesRead = jokesFile.readBytesUntil('\n', buffer, bufferSize - 1);
        buffer[bytesRead] = '\0';
        // Trim potential carriage return
        if (bytesRead > 0 && buffer[bytesRead - 1] == '\r') {
            buffer[bytesRead - 1] = '\0';
        }
    } else {
        buffer[0] = '\0';
    }

    jokesFile.close();
}

void playMusicFromSD(Robot& robot, const char* filename, int buzzerPin) {
    if (!robot.sdCardReady) {
        if (DEBUG_MODE) {
            Serial.println(F("SD Card not ready for playMusicFromSD."));
        }
        return;
    }
    File musicFile = SD.open(filename);
    if (!musicFile) {
        if (DEBUG_MODE) {
            Serial.print(F("Error opening music file "));
            Serial.println(filename);
        }
        return;
    }

    if (DEBUG_MODE) {
        Serial.print(F("Playing music from "));
        Serial.println(filename);
    }

    char lineBuffer[64];
    while (musicFile.available()) {
        int bytesRead = musicFile.readBytesUntil('\n', lineBuffer, sizeof(lineBuffer) - 1);
        lineBuffer[bytesRead] = '\0';
        
        if (bytesRead > 0 && lineBuffer[bytesRead-1] == '\r') {
            lineBuffer[bytesRead-1] = '\0';
        }

        if (strlen(lineBuffer) == 0) continue;

        char* comma = strchr(lineBuffer, ',');
        if (comma == NULL) {
            if (DEBUG_MODE) {
                Serial.print(F("Invalid music line: "));
                Serial.println(lineBuffer);
            }
            continue;
        }

        *comma = '\0';
        char* freqStr = lineBuffer;
        char* durationStr = comma + 1;

        int frequency = atoi(freqStr);
        int duration = atoi(durationStr);

        if (frequency > 0 && duration > 0) {
            tone(buzzerPin, frequency, duration);
            delay(duration + 10); // This is blocking, as noted in review
        } else if (duration > 0) {
            noTone(buzzerPin);
            delay(duration); // This is blocking
        }
    }

    noTone(buzzerPin);
    musicFile.close();
    if (DEBUG_MODE) {
        Serial.println(F("Finished playing music."));
    }
}

void listMusicFiles(Robot& robot, void (*callback)(const char* filename)) {
    if (!robot.sdCardReady) {
        if (DEBUG_MODE) {
            Serial.println(F("SD Card not ready for listMusicFiles."));
        }
        return;
    }
    File root = SD.open("/");
    if (!root) {
        if (DEBUG_MODE) {
            Serial.println(F("Failed to open SD card root directory."));
        }
        return;
    }

    if (DEBUG_MODE) {
        Serial.println(F("Listing music files:"));
    }

    while (true) {
        File entry = root.openNextFile();
        if (!entry) {
            break;
        }

        if (!entry.isDirectory()) {
            const char* entryName = entry.name();
            if (endsWith(entryName, ".txt")) {
                if (DEBUG_MODE) {
                    Serial.print(F("- "));
                    Serial.println(entryName);
                }
                callback(entryName);
            }
        }
        entry.close();
    }
    root.close();
}

