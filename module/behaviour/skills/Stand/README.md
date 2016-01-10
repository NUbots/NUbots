Stand
=============

## Description

Defaults the action to standing position.

## Usage

Include this module to allow the robot to stand when it is doing nothing.

## Consumes

* `message::AllServoWaypointsComplete` indicates that a script has finished

## Emits

* `message::ExecuteScript` instructs the script engine to run a script

## Dependencies

* The Script Engine module is required to execute the scripts
* The Darwin Motion Manager module is required to notify when scripts finish
