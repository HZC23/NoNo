#include "hardware.h"
#include "robot.h"

bool setupSDCard() {
    return true;
}

void getRandomJokeFromSD(Robot& robot, const char* filename, char* buffer, size_t bufferSize) {
    if (bufferSize > 0) {
        buffer[0] = '\0';
    }
}

bool loadConfig(Robot& robot) {
    return true;
}
