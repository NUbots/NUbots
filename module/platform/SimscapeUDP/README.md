SimscapeUDP
======

## Description
    Middleware between [Simscape](https://github.com/ElderNoSpace/Simscape/tree/murtagh/UDP) and NUbots codebase
    May not work in Ubuntu versions prior to 20.04

## Usage
    Add to a Role to pass ServoTargets messages to the Matlab NUbots/Simscape which then simulates the robot. The simulation also generates sensor data from the Servo and emits them as Servos messages.

## Emits
    DarwinSensors/Servos
    ServoTargets in the UDP scope

## Dependencies
