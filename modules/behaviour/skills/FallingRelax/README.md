Falling Relax
=============

## Description

Makes the robot relax its motors when it detects that it is falling.

## Usage

Include this module to allow the robot to relax its motors to prevent damage when falling.

## Consumes

* `message::Sensors` containing the orientation matrix and accelerometer readings

## Emits

* `message::ExecuteScript` instructs the script engine to run a script

## Dependencies

* The Script Engine module is required to execute the scripts
* The Filtered Sensors module is required to retrieve the current orientation
