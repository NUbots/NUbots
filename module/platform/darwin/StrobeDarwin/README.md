Party Darwin
============

## Description

Stores Darwin's LED colours to the beat.

## Usage

StrobeDarwin strobes from when it is installed until the program exits. It requires beat inputs but doesn't require any further action.

## Emits

* `message::DarwinSensor::HeadLED` to set head LED colour.
* `message::DarwinSensor::EyeLED` to set eye LED colour.

## Dependencies

* The Darwin HardwareIO module must be installed to set the LED colours.
* A Beat emitting module must be installed
