Party Darwin
============

## Description

A test module that repeatedly changes the Darwin's LED colours to show that
the program is running, is successfully communicating with the robot and has
not crashed.

## Usage

PartyDarwin parties from when it is installed ("night") until the program exits
("dawn"). It does not require any further action, though good music certainly
helps.

## Emits

* `message::DarwinSensor::HeadLED` to set head LED colour.
* `message::DarwinSensor::EyeLED` to set eye LED colour.

## Dependencies

* The Darwin HardwareIO module must be installed to set the LED colours.

