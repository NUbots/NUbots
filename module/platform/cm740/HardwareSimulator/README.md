# CM740 Hardware Simulator

## Description

This module simulates a CM740 subcontroller and connected sensors.

## Usage

This module simulates data that may be received from a CM740. It updates 90 times per second and
emits its simulated data as a `message::platform::RawSensors` object. This includes the CM740 error
code, LED panel, head and eye LED colour, buttons, voltage, accelerometer, gyroscope, left and right force-sensing resistors and each servo.

To change the colour of the NUgus' head or eye LEDs, emit a
`message::platform::RawSensors::EyeLED` or `message::platform::RawSensors::HeadLED`
containing the colour you wish to set them to.

To control the NUgus' servos, use `message::motion::ServoTarget`. You may
emit these commands individually or emit several at once in a `message::motion::ServoTargets`.

## Consumes

- `message::motion::ServoTarget` requesting a single servo command be performed
- `message::motion::ServoTargets` requesting a batch of servo commands be performed

## Emits

- `message::platform::RawSensors` containing the current status of the simulated NUgus
