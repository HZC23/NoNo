# Gemini

## Project: NonoController

### Overview

NonoController is an Android application designed to control an Arduino-based robot named "Nono" using Bluetooth Low Energy (BLE). The application provides a user interface to manually control the robot, switch between different autonomous modes, and monitor sensor data received from the robot.

### Technical Details

*   **Platform:** Android
*   **Language:** Kotlin
*   **UI:** Jetpack Compose
*   **Build System:** Gradle
*   **Connectivity:** Bluetooth Low Energy (BLE)

### Features

*   **Bluetooth Connection:** The app scans for nearby BLE devices, specifically targeting DFRobot Bluno boards, and establishes a connection. It displays the current connection status.
*   **Robot Control:**
    *   **Manual Control:** Provides directional buttons (Up, Down, Left, Right) for manual robot movement, including a Stop button. The directional buttons support press-and-hold for continuous movement.
    *   **Autonomous Modes:** The user can select from several autonomous modes:
        *   **Manual:** Default mode for manual control.
        *   **Auto:** A general autonomous operation mode.
        *   **Obstacle:** An obstacle avoidance mode, likely using an ultrasonic sensor.
        *   **PIR:** A mode that reacts to motion detected by a PIR sensor.
        *   **Cap:** A mode to maintain a specific compass heading (cap). The user can input a target angle.
*   **Sensor Monitoring:**
    *   **Serial Monitor:** A dedicated view to display all raw serial data coming from the robot, useful for debugging.
    *   **Dashboard:** A panel that visualizes data from the robot's sensors:
        *   **Speed:** Shows the current motor speed with a progress bar.
        *   **Distance:** Displays the distance measured by an ultrasonic sensor, with a progress bar and a visual warning for close objects.
        *   **Compass (Cap):** Shows the current and target compass heading.
        *   **Battery:** Displays the robot's battery level.
        *   **PIR Status:** Shows the status of the PIR motion sensor.
        *   **LCD Display:** Mirrors text shown on the robot's LCD screen.
*   **User Interface:**
    *   The application runs in a full-screen, immersive landscape orientation.
    *   The layout is organized into panels for connection status, directional controls, mode selection, and sensor data.

### Future Enhancements

The robot's firmware has been updated to support a 2-axis turret (horizontal and vertical) and a more advanced 3x3 environment scanning. The Android application could be enhanced to leverage these new capabilities:

*   **Turret Control:**
    *   Add a new joystick or up/down buttons to control the vertical angle of the turret.
    *   This would allow the user to manually inspect the environment at different heights.

*   **Enhanced Sensor View:**
    *   Modify the dashboard to display the 3x3 grid of distances from the new scanning function. This could be a simple grid of text values or a more graphical representation with colors to indicate distance.
    *   This would give the user a much better understanding of the robot's surroundings and help in making navigation decisions.

*   **New Commands:**
    *   Implement new Bluetooth commands to send vertical turret angle commands to the robot.
    *   The app would need to be updated to parse and display the new 3x3 distance data sent by the robot.