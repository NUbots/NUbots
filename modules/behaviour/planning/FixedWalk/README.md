Getup
=============

## Description

Plans a walk command for a walk strategy.

## Usage

Include this module to allow the robot to plan a walk command for a walk strategy.

## Consumes

* `message::behaviour::MotionCommand` containing the walk type
* `message::localisation::*` containing navigation data

## Emits

* `message::notyetimplemented` instructs the walk reflex to do a walk

## Dependencies

see consumes
