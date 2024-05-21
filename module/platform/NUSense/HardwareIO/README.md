# NUSense HardwareIO

## Description

This module is responsible for communicating with the NUgus robot's NUSense controller.

## Usage

NUSense handles reading and writing to the Dynamixel devices, and reads its own accelerometer and gyroscope (IMU). This module sends target requests for Dynamixel devices and receive the NUSense data on Dynamixel device states and IMU state.

This module is built when the subcontroller CMake flag is set to `NUSense`. Using `platform::${SUBCONTROLLER}::HardwareIO` will use this module if the subcontroller CMake flag is set to `NUSense`.

## Consumes

- `message::actuation::ServoTarget` requesting a single servo command be performed
- `message::actuation::ServoTargets` requesting a batch of servo commands be performed

## Emits

- `message::platform::NUSense` with a `DIRECT` scope so that the message is picked up locally in this module to be converted to RawSensors.
- `message::input::RawSensors` containing the current NUgus sensor data from the NUSense device.

## Dependencies
