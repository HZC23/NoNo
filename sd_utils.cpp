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
    // Initialize SPI bus with specific pins
    SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS_PIN);
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

        // --- Start Text Cleaning ---
        char* start = buffer;
        // Trim leading whitespace
        while (isspace((unsigned char)*start)) {
            start++;
        }

        // Trim trailing whitespace
        char* end = start + strlen(start) - 1;
        while (end > start && isspace((unsigned char)*end)) {
            *end-- = '\0';
        }

        // Remove leading quote
        if (*start == '"') {
            start++;
        }
        // Remove trailing quote
        end = start + strlen(start) - 1;
        if (end >= start && *end == '"') {
            *end = '\0';
        }

        // Remove trailing ellipsis "..."
        end = start + strlen(start) - 1;
        size_t len = strlen(start);
        if (len >= 3 && strcmp(start + len - 3, "...") == 0) {
            *(start + len - 3) = '\0';
        }

        // Move the cleaned text to the beginning of the buffer
        memmove(buffer, start, strlen(start) + 1);
        // --- End Text Cleaning ---

    } else {
        buffer[0] = '\0';
    }

    jokesFile.close();
}

