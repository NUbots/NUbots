# StandStill

## Description

Makes the robot stand still and not move. It emits a zero walk command until `Stability` is `STANDING` and then emits the Stand script.

## Usage

Add this module and emit a `message::strategy::StandStill` Task to make the robot stand still.

## Consumes

- `message::strategy::StandStill` a Task requesting the robot stands still
- `message::behaviour::state::Stability` indicating the stability state of the robot, to check if it is standing

## Emits

- `message::skill::Walk` to make the walk engine stop cleanly
- `message::actuation::LimbsSequence` with a Stand script loaded to make the robot stand still

## Dependencies

- The script utility
- The walk engine
