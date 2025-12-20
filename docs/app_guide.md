# Nono Robot - App Communication Guide

This guide describes the communication protocol between the Nono robot and the Android application.

## 1. Connection

The robot uses Bluetooth Serial for communication. The app should connect to the robot using the Bluetooth Serial Port Profile (SPP). The robot's Bluetooth name is "Nono".

## 2. Command Format

All commands sent to the robot must be terminated with a newline character (\n). The commands are case-insensitive.

The general command format is:

`COMMAND:VALUE\n`

For example, to set the robot's speed, the command would be:

`S:200\n`

Some commands don't require a value. For example, to toggle the obstacle avoidance mode, the command is:

`AVOID\n`

## 3. Commands

Here is a list of all the available commands:

### Movement

| Command | Description |
|---|---|
| `V:value;D:value` | Sets the robot's velocity and direction. `V` is the velocity from -100 to 100. `D` is the direction from -100 to 100. | `V:50;D:25\n` |
| `S:value` | Sets the robot's target speed. `value` is an integer from 0 to 255. | `S:200\n` |

### Modes

| Command | Description |
|---|---|
| `M:AVOID` | Switches to obstacle avoidance mode. |
| `M:SENTRY` | Switches to sentry mode. |
| `M:SCAN3D` | Starts a 3D scan. |
| `M:IDLE` | Switches to idle mode. |
| `AVOID` | Toggles obstacle avoidance mode. |
| `SENTRY` | Toggles sentry mode. |
| `SCAN3D` | Starts a 3D scan. |

### Actions

| Command | Description |
|---|---|
| `HL:ON` | Turns the headlight on. |
| `HL:OFF` | Turns the headlight off. |
| `MUSIC:PLAY` | Plays music from the SD card. |
| `MUSIC:STOP` | Stops the music. |

## 4. Telemetry

The robot sends telemetry data to the app as a JSON object, terminated with a newline character (\n). The telemetry is sent periodically.

Here is an example of the telemetry data:

```json
{"state":"IDLE","heading":180,"distance":50,"distanceLaser":25,"battery":80,"speedTarget":200}
```

### Telemetry Fields

| Field | Description |
|---|---|
| `state` | The current state of the robot. Possible values are: "IDLE", "MOVING_FORWARD", "MOVING_BACKWARD", "TURNING_LEFT", "TURNING_RIGHT", "FOLLOW_HEADING", "MAINTAIN_HEADING", "OBSTACLE_AVOIDANCE", "SCANNING", "UNKNOWN". |
| `heading` | The robot's current heading in degrees (0-360). |
| `distance` | The distance to the nearest obstacle in cm, measured by the ultrasonic sensor. |
| `distanceLaser` | The distance to the nearest obstacle in cm, measured by the laser sensor. |
| `battery` | The battery level in percent (0-100). |
| `speedTarget` | The robot's target speed (0-255). |
