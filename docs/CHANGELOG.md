# Changelog

All notable changes to this project will be documented in this file.

## [Unreleased]

### Added
- Created `Constants.java` in the Android app to centralize command strings and improve code maintainability.
- Added new string resources to `strings.xml` for improved localization and removal of hardcoded text in Java files.
- Added `WAITING_FOR_TURRET` state to Arduino's `RobotState` enum in `config.h`.
- Added `turretMoveStartTime`, `nextStateAfterTurretMove`, and `obstacleAvoidanceState` to Arduino's `Robot` struct in `state.h` to support non-blocking obstacle avoidance.
- Added new constants to Arduino's `config.h` for various timeouts, angles, and buffer sizes, replacing magic numbers throughout the code.

### Changed
- **Android App (`MainViewModel.java`)**:
    - Fixed a potential memory leak by properly removing `telemetryObserver` in `onCleared()`.
    - Replaced all hardcoded command strings with references to `Constants.java`.
- **Android App (`MainActivity.java`)**:
    - Implemented dynamic dark mode switching and layout inversion, removing the need for an app restart after changing these settings.
    - Replaced hardcoded UI strings (dialog titles, messages, button texts, and toast messages) with references to `strings.xml` resources.
    - Replaced hardcoded directional strings with constants from `Constants.java`.
- **Arduino Code (`fonctions_motrices.h`)**:
    - Refactored the `OBSTACLE_AVOIDANCE` state machine to be non-blocking, eliminating `delay()` calls for turret movements and improving robot responsiveness.
    - Updated `changeState` function to reset `obstacleAvoidanceState` when entering `OBSTACLE_AVOIDANCE`.
    - Replaced magic numbers (e.g., turning timeouts, quick scan angles, backup durations, sentry flash intervals) with constants defined in `config.h`.
- **Arduino Code (`NoNo.ino`)**:
    - Replaced hardcoded VL53L1X timing budget and inter-measurement period with constants from `config.h`.
- **Arduino Code (`support.h`)**:
    - Replaced hardcoded battery voltage min/max values with constants from `config.h`.
- **Arduino Code (`telemetry.h`)**:
    - Replaced hardcoded JSON document size with a constant from `config.h`.
- **Arduino Code (`terminal.h`)**:
    - Replaced hardcoded command buffer size and LCD text length with constants from `config.h`.

### Removed
- `BluetoothLeService_Fixed.java` (duplicate and unnecessary file).

### Fixed
- Character encoding issues in various hardcoded French strings within the Android app's `MainActivity.java` by moving them to `strings.xml`.
- Potential memory leak in Android app's `MainViewModel`.