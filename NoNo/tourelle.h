#ifndef TOURELLE_H
#define TOURELLE_H

#include <Arduino.h>
#include <Servo.h>

class Tourelle {
public:
    Tourelle(int pinHorizontal, int pinVertical);
    void attach();
    void write(int angleHorizontal, int angleVertical);
    void setPosition(int angleHorizontal, int angleVertical);
    void resetPosition();

private:
    Servo servoHorizontal;
    Servo servoVertical;
    int pinHorizontal;
    int pinVertical;
    int currentAngleHorizontal;
    int currentAngleVertical;
};

#endif // TOURELLE_H