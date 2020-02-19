Getup
=============

## Description

Allows the robot to get up when it detects that it has fallen over.

## Usage

Include this module to allow the robot to get up after it has fallen over.

## Consumes

* `message::Sensors` containing the orientation matrix and accelerometer readings
* `message::AllServoWaypointsComplete` indicates that a script has finished

## Emits

* `message::ExecuteScript` instructs the script engine to run a script

## Dependencies

* The Script Engine module is required to execute the scripts
* The Darwin Motion Manager module is required to notify when scripts finish
* The Filtered Sensors module is required to retrieve the current orientation
