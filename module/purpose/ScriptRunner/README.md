# Script Runner

## Description

Allows scripts to be run from the command line.

## Usage

ScriptRunner will attempt to run each command line parameter as a script. Once
the first script has completed it will run the next, and so on until all are
finished at which point it terminates the program.

## Consumes

- `CommandLineArguments` containing the list of scripts from the command line

## Emits

- `message::actuation::LimbsSequence` through the Script extension, which populates the requested Script/s into LimbsSequences.

## Dependencies
