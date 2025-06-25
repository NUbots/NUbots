# KickWalk

## Description

A module that allows the robot to perform kicks while walking. This module integrates the walk engine with kicking capabilities to enable fluid transitions between walking and kicking motions without stopping the robot's forward momentum.

## Usage

Include this module to allow the robot to kick while walking. This is useful for dynamic soccer gameplay where the robot needs to kick the ball while maintaining movement.

## Consumes

- `message::skill::Kick` Task requesting the robot to kick
- `message::skill::Walk` A Task requesting to walk, containing a vector with the desired velocity target, a boolean indicating whether to kick, and the leg to use when kicking.
- `message::behaviour::state::WalkState` which contains data about the robot's current movement.

## Emits

- `message::skill::Walk` A Task requesting to walk, containing a vector with the desired velocity target, a boolean indicating whether to kick, and the leg to use when kicking.

## Dependencies
