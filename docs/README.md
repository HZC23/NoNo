# NoNo Robot Project

This repository contains the source code for the "NoNo" robot's Arduino firmware, built using the PlatformIO IDE.

NoNo is a differential-drive mobile robot based on the ESP32-S3, designed for autonomous navigation and remote control.

## Documentation

- **[Software](./software.md):** A detailed description of the software architecture, state machine, and main libraries.
- **[Hardware](./hardware.md):** A list of the electronic components and their wiring.
- **[Communication](./communication.md):** The serial communication protocol.
- **[Commands](./commands.md):** An exhaustive list of serial commands to control the robot.
- **[Calibration](./calibration.md):** Detailed instructions for calibrating the magnetometer.
- **[App Guide](./app_guide.md):** A guide for connecting to and controlling the robot via a serial terminal or application.

## Main Features

- **Non-blocking architecture:** The firmware is based on a state machine for reactive behavior.
- **Multiple navigation modes:** Manual control, heading tracking (GOTO), and autonomous obstacle avoidance.
- **Sensors:**
    - Magnetometer/Accelerometer (LSM303) for orientation.
    - Ultrasonic distance sensor for obstacle avoidance.
    - Time-of-Flight laser distance sensor (VL53L1X) for precise measurements.
- **Actuators:**
    - Two DC motors with an MX1508 driver for propulsion.
    - A pan/tilt turret with two servos.
- **User Interface:**
    - I2C LCD screen to display status and sensor data.
    - Serial communication via USB for control and telemetry.

## Development

### Prerequisites

- [Visual Studio Code](https://code.visualstudio.com/)
- [PlatformIO IDE extension](https://platformio.org/install/ide?install=vscode) for VSCode.

### Compilation and Uploading

This project is configured to be built with PlatformIO.

1.  Open the project folder in Visual Studio Code.
2.  The PlatformIO extension should recognize the `platformio.ini` file.
3.  To compile the firmware, use the "Build" task in the PlatformIO project task list or run the following command in the PlatformIO CLI terminal:
    ```bash
    platformio run
    ```
4.  To upload the firmware to the ESP32-S3 board, use the "Upload" task or run:
    ```bash
    platformio run --target upload
    ```
PlatformIO will automatically handle the correct board type (`freenove_esp32_s3_lite`) and download all necessary library dependencies.