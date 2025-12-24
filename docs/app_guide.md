# Nono Robot - App & Serial Communication Guide

This guide describes the serial communication protocol for controlling the Nono robot.

## 1. Connection

The robot communicates via **Serial (USB)**. A serial terminal or companion app can connect to the robot with the following settings:

*   **Baud Rate:** `115200`
*   **Data Bits:** 8
*   **Parity:** None
*   **Stop Bits:** 1

**Connection Flow:**
1.  Establish a serial connection to the robot's USB port.
2.  Once connected, commands can be sent, and telemetry is received periodically.

## 2. Command Format

All commands sent to the robot must be terminated with a newline character (`\n`). The command prefixes are case-insensitive.

The general format is `KEY:VALUE\n`.

## 3. Command Reference

| Prefix | Parameters | Description | Example |
| :--- | :--- | :--- | :--- |
| `M` | `velocity,turn` | **Move**: Directly controls the motors. `velocity` is forward/backward (-255 to 255), `turn` is the turning component (-255 to 255). Non-sticky. | `M:150,50` |
| `G` | `heading` | **Goto**: Puts the robot in `FOLLOW_HEADING` state to turn to the specified absolute heading (0-359 degrees). | `G:90` |
| `S` | `speed` | **Speed**: Sets the manual target speed (`vitesseCible`) from 0-255. | `S:200` |
| `CO` | `offset` | **Compass Offset**: Applies a permanent fine-tuning offset (in degrees) to the compass. | `CO:-5.5` |
| `SM` | `XBOX` or `SERIAL` | **Set Mode**: Changes the primary control interface and reboots the robot. `XBOX` enables the controller, `SERIAL` disables it. | `SM:XBOX` |
| `E` | `mode` | **State (État)**: Changes the robot's behavior state. See modes below. | `E:AVOID` |
| `L` | `ON`, `OFF`, `TOGGLE` | **Light**: Controls the headlight. | `L:ON` |

### Available Modes for `E:` command

*   `IDLE`: Stops all actions.
*   `AVOID`: Activates `OBSTACLE_AVOIDANCE` mode.
*   `SENTRY`: Activates `SENTRY_MODE`.
*   `CALIBRATE`: Starts the 15-second compass calibration.
*   `TOGGLE_AVOID`: Toggles `OBSTACLE_AVOIDANCE` mode.
*   `TOGGLE_SENTRY`: Toggles `SENTRY_MODE`.

## 4. Telemetry

The robot sends telemetry data as a JSON object, terminated with a newline character (`\n`).

**Example:**
```json
{"state":"IDLE","heading":26,"distance":81,"distanceLaser":86,"battery":51,"speedTarget":100}
```

### Telemetry Fields

| Field | Description |
| :--- | :--- |
| `state` | The current state of the robot (e.g., "IDLE", "OBSTACLE_AVOIDANCE", "FOLLOW_HEADING"). |
| `heading` | The robot's current compass heading in degrees (0-360). |
| `distance` | Distance from the ultrasonic sensor in cm. |
| `distanceLaser` | Distance from the ToF laser sensor in cm. |
| `battery` | Battery level percentage (0-100). |
| `speedTarget` | The manually set target speed (`vitesseCible`). **Note:** This does not reflect the speed used in autonomous modes. |
