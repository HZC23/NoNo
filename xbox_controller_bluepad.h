#ifndef XBOX_CONTROLLER_BLUEPAD_H
#define XBOX_CONTROLLER_BLUEPAD_H

#include <Bluepad32.h>
#include "state.h"
#include "fonctions_motrices.h"
#include "tourelle.h"
#include "display.h"
#include "config.h"
#include <FS_MX1508.h>

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

    // Bluepad32 controller management
    static ControllerPtr myControllers[BP32_MAX_CONTROLLERS];
    static void onConnectedController(ControllerPtr ctl);
    static void onDisconnectedController(ControllerPtr ctl);
};

#endif // XBOX_CONTROLLER_BLUEPAD_H
