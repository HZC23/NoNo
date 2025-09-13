
#ifndef TOURELLE_H
#define TOURELLE_H

#include <Servo.h>
#include "config.h" // Include for constants if needed

class Tourelle {
private:
    Servo servoH;
    Servo servoV;
    int servoH_pin;
    int servoV_pin;

public:
    // Constructor with initializer list for better practice
    Tourelle(int pinH, int pinV) : servoH_pin(pinH), servoV_pin(pinV) {}

    // Attach servos to pins
    void attach() {
      servoH.attach(servoH_pin);
      servoV.attach(servoV_pin);
    }

    void detach() {
      servoH.detach();
      servoV.detach();
    }

    // Write angles to servos (restored and implemented)
    void write(int angleH, int angleV) {
      servoH.write(angleH);
      servoV.write(angleV);
    }

    // Get current angles (can be improved later)
    int getAngleHorizontal() {
        return servoH.read();
    }

    int getAngleVertical() {
        return servoV.read();
    }
};

#endif // TOURELLE_H
