Script Optimizer
============

## Description

This module is used to optize scripts. It will wait for a message to be sent
over the network with a script to perform, and will report back its statistics while executing it.
This allows a central optimzer to analyse and improve scripts as they run.

## Usage

This module will wait for communication over the network with a script to run.
It will then execute this script and respond with the sensor values over the script

## Consumes

* `message::OptimizeScript

## Emits

* `message::Script` to execute scripts
* `message::SensorValues` to the network after scripts have run

## Dependencies

* The Darwin HardwareIO module must be installed to read sensors and execute scripts
* The DarwinMotionManager must be installed to control motion
* ScriptEngine must be installed to execute scripts

