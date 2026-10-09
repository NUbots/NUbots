# HardwareIO

## Description

This module is responsible for communicating with the Booster K1 via the Booster Robotics SDK.

## Usage

This module acts as an abstraction layer between the NUbots codebase and the Booster Robotics SDK. It translates SDK
messages to NUClear Neutrons and vice-versa.


This module is built when the subcontroller CMake flag is set to `Booster`. Using `platform::${SUBCONTROLLER}::HardwareIO` will use this module if the subcontroller CMake flag is set to `Booster`.

## Consumes

- `message::booster::BoosterWalk` to send a walk velocity command to the robot.
- `message::booster::BoosterHeadRot` to rotate the head.
- `message::booster::BoosterKick` to trigger a kick.
- `message::booster::BoosterVisualKick` to trigger a vision-guided kick.
- `message::booster::BoosterGetUp` to trigger the get-up routine.
- `message::booster::BoosterMode` to change the robot's motion mode.
- `message::localisation::ResetFieldLocalisation` to reset the robot's odometry.

## Emits

- `message::platform::RawSensors` with IMU, joint, battery, and button state from the SDK's low state.
- `message::booster::BoosterOdometry` with the SDK's odometry pose.
- `message::booster::BoosterFallDownState` with the robot's fall state and whether recovery is available.
- `message::booster::BoosterModeState` with the robot's current motion mode.

## Dependencies

- Booster Robotics SDK
