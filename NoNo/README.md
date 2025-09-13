# NoNo Robot Project

## Overview
The NoNo project is an Arduino-based robot designed for various autonomous tasks. It utilizes a combination of sensors and motors to navigate and interact with its environment.

## Project Structure
- **NoNo.ino**: The main entry point of the Arduino project. Initializes global variables, sets up hardware components, and contains the main loop for sensor updates and state machine logic.
- **fonctions.h**: Contains function declarations and type definitions related to the robot's functionalities.
- **fonctions_motrices.h**: Defines motor control functions and constants, including state management for the robot.
- **sensor_task.h**: Manages sensor-related tasks, including initialization and non-blocking updates for sensors.
- **tourelle.h**: Contains definitions and functions related to the turret mechanism of the robot.
- **docs/software.md**: Provides documentation on the software architecture, including state machine descriptions and task management.

## Setup Instructions
1. **Hardware Requirements**: Ensure you have all necessary hardware components, including motors, sensors, and the Arduino board.
2. **Software Requirements**: Install the Arduino IDE and any required libraries for the sensors and motors used in the project.
3. **Uploading the Code**: Open the `NoNo.ino` file in the Arduino IDE, connect your Arduino board, and upload the code.

## Usage
Once the code is uploaded, the robot will initialize its components and enter the main loop, where it will continuously update sensor readings and manage its state based on the programmed logic.

## Contributing
Feel free to contribute to the project by adding new features or improving existing functionalities. Please ensure to follow the coding standards and document any changes made.

## License
This project is open-source and available for modification and distribution under the terms of the MIT License.