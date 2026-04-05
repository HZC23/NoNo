#ifndef COMMS_H
#define COMMS_H

#include <Bluepad32.h>
#include "fonctions_motrices.h"

// Forward declarations to solve circular dependency
class MX1508;
class Tourelle;
struct Robot;



// --- Function Prototypes ---

// Terminal
void checkSerial();
void processCommand(String command);

// Telemetry
void sendTelemetry(Robot& robot);
void sendPeriodicData(Robot& robot);

// Display
void setLcdText(Robot& robot, const char* text, bool isContinuation = false);
void handleLcdAnimations(Robot& robot);
void displayRandomJoke(Robot& robot);
void displayJokesIfIdle(Robot& robot);
void updateLcdDisplay(Robot& robot);

// Haptics
void trigger_rumble(uint8_t duration, uint8_t weak_magnitude, uint8_t strong_magnitude);
void toggle_msc_mode();

// --- Xbox Controller Class ---
class XboxControllerBluepad {
public:
    explicit XboxControllerBluepad(Robot& robotRef);
    void begin();
    void processControllers();
    bool isConnected();
    static ControllerPtr getController(int index); // New getter

private:
    Robot& robot;
    static Robot* robot_ptr;
    static ControllerPtr myControllers[BP32_MAX_CONTROLLERS];
    static void onConnectedController(ControllerPtr ctl);
    static void onDisconnectedController(ControllerPtr ctl);
};

extern XboxControllerBluepad xboxController;

#endif // COMMS_H
