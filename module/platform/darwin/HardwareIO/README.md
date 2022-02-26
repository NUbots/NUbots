# Darwin Hardware I/O

## Description

This module is responsible for communicating with the Darwin robot's CM740
controller.

## Usage

Darwin Hardware I/O connects at startup to the Darwin controller located on
`/dev/CM740`. If this does not succeed an exception is thrown and startup is
aborted.

This module reads the current status of the Darwin 90 times per second and
emits it as a `message::platform::RawSensors` object. This includes the CM740 error
code, LED panel, head and eye LED colour, buttons, voltage, accelerometer,
gyroscope, left and right force-sensing resistors and each servo.

To change the colour of the Darwin's head or eye LEDs, emit a
`message::platform::RawSensors::EyeLED` or `message::platform::RawSensors::HeadLED`
containing the colour you wish to set them to.

To control the Darwin's servos, use `message::motion::ServoTarget`. You may
emit these commands individually or emit several at once in a `message::motion::ServoTargets`.

## Consumes

- `message::platform::RawSensors::EyeLED` requesting a change to eye LED colour
- `message::platform::RawSensors::HeadLED` requesting a change to head LED colour
- `message::motion::ServoTarget` requesting a single servo command be performed
- `message::motion::ServoTargets` requesting a batch of servo commands be performed

## Emits

- `message::platform::RawSensors` containing the current status of the Darwin

## Dependencies

- The USB TTY communication relies on Linux-specific system calls
