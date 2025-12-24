#ifndef COMMS_H
#define COMMS_H

#include "robot.h"
#include <Bluepad32.h>
#include "fonctions_motrices.h"

// Forward declarations to solve circular dependency
class MX1508;
class Tourelle;


// --- Function Prototypes ---

// Terminal (from terminal.h)
void checkSerial();
void processCommand(String command);

// Telemetry (from telemetry.h)
void sendTelemetry(Robot& robot);
void sendPeriodicData(Robot& robot);
const char* stateToString(RobotState state);

// Display (from display.h)
void setLcdText(Robot& robot, const char* text, bool isContinuation = false);
void handleLcdAnimations(Robot& robot);
void displayRandomJoke(Robot& robot);
void displayJokesIfIdle(Robot& robot);
void updateLcdDisplay(Robot& robot);

// --- Xbox Controller Class ---
class XboxControllerBluepad {
public:
    explicit XboxControllerBluepad(Robot& robotRef);
    void begin();
    void processControllers();
    bool isConnected();
    void setHardware(MX1508& motorA_ref, MX1508& motorB_ref, Tourelle& tourelle_ref);

private:
    Robot& robot;
    static Robot* robot_ptr;
    static MX1508* motorA_ptr;
    static MX1508* motorB_ptr;
    static Tourelle* tourelle_ptr;

    static ControllerPtr myControllers[BP32_MAX_CONTROLLERS];
    static void onConnectedController(ControllerPtr ctl);
    static void onDisconnectedController(ControllerPtr ctl);
};

extern XboxControllerBluepad xboxController;

#endif // COMMS_H
