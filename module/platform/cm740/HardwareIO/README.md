# CM740 Hardware I/O

## Description

This module is responsible for communicating with a CM740 controller.

## Usage

CM740 Hardware I/O connects at startup to the CM740 controller located on
`/dev/CM740`. If this does not succeed an exception is thrown and startup is
aborted.

This module reads the current status of the CM740 90 times per second and
emits it as a `message::platform::RawSensors` object. This includes the CM740 error
code, LEDs, buttons, voltage, accelerometer, gyroscope, left and right
force-sensing resistors and each servo. CM740s have the ability to connect to head and eye LEDs and force-sensing resistors which are
referred to in this module and can be read from the CM740,
however the NUgus robot does not utilise these capabilities at present.

Emit a `message::platform::RawSensors::EyeLED` or `message::platform::RawSensors::HeadLED`
with a colour to give instructions to the CM740 to change the eye or head LED colour.

To control the servos, use `message::motion::ServoTarget`. You may
emit these commands individually or emit several at once in a `message::motion::ServoTargets`.

## Consumes

- `message::motion::ServoTarget` requesting a single servo command be performed
- `message::motion::ServoTargets` requesting a batch of servo commands be performed

## Emits

- `message::platform::RawSensors` containing the current status of the NUgus

## Dependencies

- The USB TTY communication relies on Linux-specific system calls
