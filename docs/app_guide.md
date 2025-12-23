# Nono Robot - App Communication Guide

This guide describes the communication protocol between the Nono robot and its companion application.

## 1. Connection

The robot communicates via **Serial (USB)**. The app should connect to the robot's serial port with the following settings:

*   **Baud Rate:** `115200` (configurable in `config.h` as `SERIAL_BAUD_RATE`)
*   **Data Bits:** 8
*   **Parity:** None
*   **Stop Bits:** 1
*   **Flow Control:** None

**Connection Flow:**
1.  Establish a serial connection to the robot's USB port.
2.  Once connected, commands can be sent, and telemetry can be received.

## 2. Command Format

All commands sent to the robot must be terminated with a newline character (`\n`). The commands are case-insensitive.

The general command format is:

`COMMAND:VALUE\n`

For example, to set the robot's speed, the command would be:

`S:200\n`

Some commands don't require a value. For example, to toggle the obstacle avoidance mode, the command is:

`AVOID\n`

Manual control commands (like `V:value;D:value`) are non-sticky and must be sent continuously to maintain movement.

## 3. Commands

Here is a list of all the available commands:

### Movement

| Command | Description | Example |
|---|---|---|
| `V:value;D:value` | Sets the robot's velocity and direction using differential steering. `V` is the velocity from -100 to 100. `D` is the direction from -100 to 100. | `V:50;D:25\n` |
| `S:value` | Sets the robot's target speed. `value` is an integer from 0 to 255. | `S:200\n` |

### Modes

| Command | Description |
|---|---|
| `M:IDLE` | Switches to idle mode. The robot stops all autonomous tasks. |
| `M:FOLLOW_HEADING` | Activates autonomous navigation to follow a specific heading (GOTO mode). |
| `M:SMART_AVOIDANCE` | Activates general roaming with intelligent obstacle avoidance. |
| `M:AVOID` | Switches to obstacle avoidance mode. (Alias for `M:SMART_AVOIDANCE`) |
| `M:SENTRY` | Switches to sentry mode. |
| `AVOID` | Toggles obstacle avoidance mode. (Legacy command, prefer `M:SMART_AVOIDANCE`)|
| `SENTRY` | Toggles sentry mode. (Legacy command, prefer `M:SENTRY`) |


### Configuration & Calibration

| Command | Description | Example |
|---|---|---|
| `CMD:COMPASS_OFFSET:value` | Sets the fine-tuning offset for the compass. `value` is an integer representing the offset. | `CMD:COMPASS_OFFSET:10\n` |


### Actions

| Command | Description |
|---|---|
| `HL:ON` | Turns the headlight on. |
| `HL:OFF` | Turns the headlight off. |

## 4. Telemetry

The robot sends telemetry data to the app as a JSON object, terminated with a newline character (`\n`). The telemetry is sent periodically.

Here is an example of the telemetry data:

```json
{"state":"IDLE","heading":180,"distance":50,"distanceLaser":25,"battery":80,"speedTarget":200}
```

### Telemetry Fields

| Field | Description |
|---|---|
| `state` | The current state of the robot. Possible values include: "IDLE", "MOVING_FORWARD", "MOVING_BACKWARD", "TURNING_LEFT", "TURNING_RIGHT", "FOLLOW_HEADING", "MAINTAIN_HEADING", "OBSTACLE_AVOIDANCE", "AVOID_MANEUVER", "SCANNING", "UNKNOWN". |
| `heading` | The robot's current heading in degrees (0-360), derived from the LSM303 compass. |
| `distance` | The distance to the nearest obstacle in cm, measured by the ultrasonic sensor. |
| `distanceLaser` | The distance to the nearest obstacle in cm, measured by the VL53L1X laser sensor. |
| `battery` | The battery level in percent (0-100). |
| `speedTarget` | The robot's target speed (0-255). |
