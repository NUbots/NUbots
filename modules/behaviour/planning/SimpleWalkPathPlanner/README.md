Simple Walk Path Planner
=============

## Description

Plans a simple path trajectory to the ball, in a straight line. It is simplified as it does not attempt to line up with the goals.

## Usage

Include this module to allow the robot to plan a walk command for a walk strategy.

## Consumes

* `messages::behaviour::MotionCommand` containing the walk type
* `messages::localisation::*` containing navigation data

## Emits

* `messages::notyetimplemented` instructs the walk reflex to do a walk

## Dependencies

see consumes
