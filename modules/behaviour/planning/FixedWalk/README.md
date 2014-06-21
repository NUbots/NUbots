Getup
=============

## Description

Plans a walk command for a walk strategy.

## Usage

Include this module to allow the robot to plan a walk command for a walk strategy.

## Consumes

* `messages::behaviour::WalkStrategy` containing the walk type
* `messages::localisation::*` containing navigation data

## Emits

* `messages::notyetimplemented` instructs the walk reflex to do a walk

## Dependencies

see consumes
