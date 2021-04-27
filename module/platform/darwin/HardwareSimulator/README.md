NUgus Hardware I/O
===================

## Description

This module is responsible for communicating with the NUgus' CM740 controller.

## Usage

NUgus Hardware I/O connects at startup to the NUgus controller located on
`/dev/usbTTY0`. If this does not succeed an exception is thrown and startup is
aborted.

This module reads the current status of the NUgus 50 times per second and
emits it as a `message::platform::darwin::DarwinSensors` object. This includes the CM740 error
code, LED panel, head and eye LED colour, buttons, voltage, accelerometer,
gyroscope, left and right force-sensing resistors and each servo.

To change the colour of the NUgus' head or eye LEDs, emit a
`message::platform::darwin::DarwinSensors::EyeLED` or `message::platform::darwin::DarwinSensors::HeadLED`
containing the colour you wish to set them to.

To control the NUgus' servos, use `message::behaviour::ServoCommand`.

## Consumes

* `message::platform::darwin::DarwinSensors::EyeLED` requesting a change to eye LED colour
* `message::platform::darwin::DarwinSensors::HeadLED` requesting a change to head LED colour
* `message::behaviour::ServoCommand` requesting a single servo command be performed

## Emits

* `message::platform::darwin::DarwinSensors` containing the current status of the NUgus

## Dependencies

* The USB TTY communication relies on Linux-specific system calls
