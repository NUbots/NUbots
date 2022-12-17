# FallingRelaxPlanner

## Description

The FallingRelaxPlanner module is used to detect when the robot is actively falling over and run a relax action to remove torque to the servos.
This is done to prevent damage when the robot impacts the ground, hopefully saving many gear set changes

## Usage

Emit a `message::plan::RelaxWhenFalling` as a Task.
Make sure that it has a lower challenge priority than the GetUp, otherwise it may trigger in the middle of the getup resulting in an infinite loop of falling and half getting up.

## Consumes

- `message::input::Sensors` to determine when the robot is actively falling over
- `message::plan::RelaxWhenFalling` as a Task to activate this module allowing it to relax when the robot is falling over

## Emits

- `message::actuation::BodySequence` containing the relax script

## Dependencies

- The Script extension
