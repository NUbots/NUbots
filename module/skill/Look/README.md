# Look

## Description

Moves the head to the requested position, taking into account smoothing.

## Usage

Include this module to move the head.

## Consumes

- `message::skill::Look` a Task requesting to look somewhere
- `message::input::Sensors` to get the current head angles for smoothing

## Emits

- `message::actuation::HeadIK` to calculate and request head joint angles

## Dependencies

- The coordinates utility for spherical to cartesian conversion
