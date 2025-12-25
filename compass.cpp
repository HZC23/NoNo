#include "hardware.h"
#include "robot.h"

void compass_init(Robot& robot) {
}

float getCalibratedHeading(Robot& robot) {
    return 0.0f;
}

void calibrateCompass(Robot& robot) {
}

void saveCompassCalibration(const Robot& robot) {
}

void loadCompassCalibration(Robot& robot) {
}

bool isCompassCalibrationValid() {
    return true;
}

float calculateHeading(float y, float x) {
    return 0.0f;
}

float calculateHeading(const LSM303& compass) {
    return 0.0f;
}

float getPitch(Robot& robot) {
    return 0.0f;
}

void displayCompassInfo(Robot& robot) {
}

bool detectImpactOrStall(Robot& robot) {
    return false;
}

void Mcap(Robot& robot, int n) {
}
