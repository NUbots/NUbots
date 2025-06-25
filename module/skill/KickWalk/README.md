# KickWalk

## Description

Allows the robot to perform kicks while walking. This module uses the walk engine to make a single large step when a kick task is emitted. The kicks integrate seamlessly with the robot's regular walk.

## Usage

Include this module to allow the robot to kick while walking. This is useful for dynamic soccer gameplay where the robot needs to kick the ball while maintaining movement.

## Consumes

- `message::skill::Kick` Task requesting the robot to kick
- `message::skill::Walk` A Task requesting to walk, containing a vector with the desired velocity target, a boolean indicating whether to kick, and the leg to use when kicking.
- `message::behaviour::state::WalkState` which contains data about the robot's current movement.

## Emits

- `message::skill::Walk` A Task requesting to walk, containing a vector with the desired velocity target, a boolean indicating whether to kick, and the leg to use when kicking.

## Dependencies
