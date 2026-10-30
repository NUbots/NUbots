# Servos

## Description

Forwards servo commands from the Director to the Booster K1 via the Booster Robotics SDK. It provides the
`message::actuation::Servos` task and converts it into a `message::booster::BoosterLowCmd`.

## Usage

Install `module::actuation::Servos` in the powerplant, and have a module that talks to the robot's low-level
controller (e.g. the Booster IO module) consume `BoosterLowCmd`.

Other modules request servo movement by emitting a `message::actuation::Servos` task. The command it carries is
passed through unchanged, so any joint limiting or smoothing needs to happen before the task is emitted. The
`Provide` runs at `Priority::HIGH` so commands reach the robot with minimal delay.

This module has no configuration file.

## Consumes

- `message::actuation::Servos` as a Director task (via `Provide`)

## Emits

- `message::booster::BoosterLowCmd`

## Dependencies

- The Director (`module/extension/Director`)
- A module that sends `BoosterLowCmd` to the robot
