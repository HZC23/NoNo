#ifndef TOURELLE_H
#define TOURELLE_H

#include <Servo.h>

class Tourelle {
public:
    Tourelle(int pinH, int pinV) {pinHorizontal=pinH; pinVertical=pinV;}

    void attach() {}

    void detach() {}

    void write(int angleH, int angleV) {}

    void writeHorizontal(int angleH) {}

    void writeVertical(int angleV) {}

    int getAngleHorizontal() {
        return 90;
    }

    int getAngleVertical() {
        return 90;
    }

private:
    Servo servoHorizontal;
    Servo servoVertical;
    int pinHorizontal;
    int pinVertical;
    int angleHorizontal;
    int angleVertical;
};

#endif // TOURELLE_H