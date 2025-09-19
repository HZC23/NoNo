# Gemini

## Project: NonoController

### Overview

NonoController is an Android application designed to control an Arduino-based robot named "Nono" using Bluetooth Low Energy (BLE). The application provides a user interface to manually control the robot, switch between different autonomous modes, and monitor sensor data received from the robot.

### Technical Details

*   **Platform:** Android
*   **Language:** Kotlin
*   **UI:** Jetpack Compose
*   **Build System:** Gradle
*   **Connectivity:** Bluetooth Low Energy (BLE) via un pont série transparent.

### Features

*   **Bluetooth Connection:** The app scans for nearby BLE devices, specifically targeting DFRobot Bluno boards, and establishes a connection. It displays the current connection status.
*   **Robot Control:**
    *   **Manual Control:** Provides directional buttons (Up, Down, Left, Right) and a Stop button. These send `CMD:MOVE:FWD`, `CMD:MOVE:BWD`, `CMD:MOVE:LEFT`, `CMD:MOVE:RIGHT`, and `CMD:MOVE:STOP` commands respectively. The directional buttons support press-and-hold for continuous movement.
    *   **Autonomous Modes:** The user can command the robot to enter specific modes:
        *   **Manual:** Default mode for direct control.
        *   **Follow Heading (Cap):** The user can input a target angle (0-359). The app sends a `CMD:GOTO:<angle>` command, and the robot will navigate to maintain that compass heading.
    *   **Other Commands:** The app can also send commands to set speed (`CMD:SPEED:<0-255>`), turn by a specific angle (`CMD:TURN:<angle>`), control the headlight (`CMD:LIGHT:ON/OFF`), and initiate compass calibration (`CMD:CALIBRATE:COMPASS`).

*   **Sensor Monitoring:**
    *   **Serial Monitor:** A dedicated view to display all raw serial data coming from the robot, useful for debugging.
    *   **Dashboard:** A panel that visualizes telemetry data sent by the robot in JSON format:
        *   **State:** Shows the robot's current state machine status (e.g., `IDLE`, `FOLLOW_HEADING`, `OBSTACLE_AVOIDANCE`).
        *   **Speed:** Shows the current and target motor speed with a progress bar.
        *   **Distance:** Displays the distance measured by an ultrasonic sensor, with a progress bar and a visual warning for close objects.
        *   **Compass (Heading):** Shows the current compass heading.
        *   **Battery:** Displays the robot's estimated battery level.
    *   *Note: The UI may have placeholders for PIR status and LCD text, but this data is not yet included in the robot's standard telemetry packet.*
*   **User Interface:**
    *   The application runs in a full-screen, immersive landscape orientation.
    *   The layout is organized into panels for connection status, directional controls, mode selection, and sensor data.

### Future Enhancements (Bridging the Gap with Firmware)

The robot's firmware (`NoNo.ino`) has capabilities that are not yet exposed through the BLE communication protocol. The Android app can be enhanced to leverage these.

*   **Turret Control:**
    *   The robot has a 2-axis turret installed (`tourelle.h`).
    *   **App Task:** Add a joystick or buttons to control the horizontal and vertical angles of the turret.
    *   **Firmware Task:** Implement new commands in `terminal.h` (e.g., `CMD:TURRET:H:<angle>` and `CMD:TURRET:V:<angle>`) to receive and act on these commands.

*   **Enhanced Environment Scanning:**
    *   The firmware includes states for environment scanning (`SCANNING_ENVIRONMENT`).
    *   **App Task:** Modify the dashboard to display a 3x3 grid of distances from the scanning function, providing a better understanding of the robot's surroundings.
    *   **Firmware Task:** Implement the `scan` logic to populate a 3x3 grid and add a new telemetry message or extend the existing JSON to send this grid data to the app.