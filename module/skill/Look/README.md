# Look

## Description

Moves the head so that the robot is looking in a given direction, with the option for smoothing.

## Usage

Include this module to make the robot look in the given direction.

## Consumes

- `message::skill::Look` a Task requesting to look in a direction
- `message::input::Sensors` to get the current head angles for smoothing

## Emits

- `message::actuation::HeadIK` to calculate and request head joint angles

## Dependencies

- The coordinates utility
