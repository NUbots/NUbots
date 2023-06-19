# OpenCR HardwareIO

## Description

This module is responsible for communicating with the NUgus robot's OpenCR
controller.

## Usage

OpenCR Hardware I/O connects at startup to the OpenCR controller and sets up the starting state of the device.

The module runs a continuous loop where it

- Sends a servo syncwrite instruction to all servos with updated instructions, if there are any new instructions
- Requests to read the servo's current state
- Processes the read data from each of the 20 servos
- Sends a OpenCR write instruction to the controller if there are any new instructions
- Requests to read the OpenCR's current state
- Processes the OpenCR read data

These are done in order on a loop and if we fail to get a return message when we expect to get one, the module attempts a reconnect the OpenCR device and requests to read the servos.

Once every loop, a RawSensors message is constructed with the current data recorded from the controller.

# Consumes

- `message::platform::RawSensors::EyeLED` requesting a change to eye LED colour
- `message::platform::RawSensors::HeadLED` requesting a change to head LED colour
- `message::platform::RawSensors::LEDPanel` requesting a change to LED panel colour
- `message::actuation::ServoTarget` requesting a single servo command be performed
- `message::actuation::ServoTargets` requesting a batch of servo commands be performed
- `message::platform::StatusReturn` used locally in the module to capture data input from the controller and process it in a separate reaction with `Sync`.

## Emits

- `message::platform::RawSensors` containing the current status of the NUgus
- `message::platform::StatusReturn` used locally in the module to capture data input from the controller and process it in a separate reaction with `Sync`.

## Dependencies

- The USB TTY communication relies on Linux-specific system calls
