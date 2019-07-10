Darwin Hardware I/O
===================

## Description

This module is responsible for communicating with the Darwin robot's CM730
controller.

## Usage

Darwin Hardware I/O connects at startup to the Darwin controller located on
`/dev/usbTTY0`. If this does not succeed an exception is thrown and startup is
aborted.

This module reads the current status of the Darwin 50 times per second and
emits it as a `message::DarwinSensors` object. This includes the CM730 error
code, LED panel, head and eye LED colour, buttons, voltage, accelerometer,
gyroscope, left and right force-sensing resistors and each servo.

To change the colour of the Darwin's head or eye LEDs, emit a
`message::DarwinSensors::EyeLED` or `message::DarwinSensors::HeadLED`
containing the colour you wish to set them to.

To control the Darwin's servos, use `message::DarwinServoCommand`. You may
emit these commands individually or emit several at once in a `std::vector`.

## Consumes

* `message::DarwinSensors::EyeLED` requesting a change to eye LED colour
* `message::DarwinSensors::HeadLED` requesting a change to head LED colour
* `message::DarwinServoCommand` requesting a single servo command be performed
* `std::vector<message::DarwinServoCommand>` requesting a batch of servo
  commands be performed

## Emits

* `message::DarwinSensors` containing the current status of the Darwin

## Dependencies

* The USB TTY communication relies on Linux-specific system calls

