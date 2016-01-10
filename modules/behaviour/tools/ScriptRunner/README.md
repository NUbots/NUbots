Script Runner
=============

## Description

Allows scripts to be run from the command line.

## Usage

ScriptRunner will attempt to run each command line parameter as a script. Once
the first script has completed it will run the next, and so on until all are
finished at which point it terminates the program.

## Consumes

* `CommandLineArguments` containing the list of scripts from the command line
* `message::AllServoWaypointsComplete` indicates that a script has finished

## Emits

* `message::ExecuteScript` instructs the script engine to run a script

## Dependencies

* The Script Engine module is required to execute the scripts
* The Darwin Motion Manager module is required to notify when scripts finish
