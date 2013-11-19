Dance Engine
============

## Description

Loads scripts containing dance moves and scales them to the tempo of music,
allowing the robot to effectively dance in time with it.

## Usage

This module loads dance scripts in the same format as the scripting engine.
Each one should be designed to last the duration of a single beat.

When the dance engine receives a `messages::Beat` it tells the robot to stand
by running the Stand.json script. Once this is complete, it chooses a dance
script at random, then scales its timing to match the tempo of the music. The
adjusted script is then sent as a `messages::ExecuteScript`.

## Consumes

* `messages::Configuration<DanceScripts>` containing the loaded scripts
* `messages::Beat` indicating a beat has occurred in the music
* `messages::AllServoWaypointsComplete` indicating that the robot has
  finished running its script and is ready to start another

## Emits

* `messages::ExecuteScript` containing a timing-adjusted dance script to run
* `messages::ExecuteScriptByName` to run the stand script

## Configuration

Dance scripts are located in the 'scripts/dance' directory. They follow the
same format as any other motion scripts, see the readme file for the Script
Engine module for details.

## Dependencies

* The Config System module is required to read script files
* The Darwin Movement Manager is required to know when scripts are finished
* The Script Engine module is used to run adjusted scripts

