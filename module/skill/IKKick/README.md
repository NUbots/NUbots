# IKKick

## Description

Performs a kick using inverse kinematics by balancing on one support foot and moving the other foot through a kick
trajectory.

The module:

- Chooses support foot from the requested kick target (`y < 0` uses left support, otherwise right)
- Transforms kick target/direction into support-foot coordinates
- Runs a balancing phase and then a kick phase
- Emits leg IK tasks each update until the kick sequence is complete

## Usage

Provide a `message::skill::Kick` task together with current `Sensors` and `KinematicsModel`.

`IKKick` runs as a behaviour skill (`Provide<Kick>`) at 90 Hz and publishes `LeftLegIK`/`RightLegIK` tasks for the
legs during the kick.

Configuration is read from `IKKick.yaml`, including:

- servo gain/torque
- balancer timing/geometry settings
- active-balance controller gains
- kick trajectory parameters (`kick_velocity`, `follow_through`, `wind_up`, etc.)
- kick frame templates (`lift_foot`, `kick`, `place_foot`)

## Consumes

- `extension::Configuration` from `IKKick.yaml`
- `message::skill::Kick`
- `message::input::Sensors`
- `message::actuation::KinematicsModel`

## Emits

- `message::actuation::LeftLegIK`
- `message::actuation::RightLegIK`
- `extension::behaviour::Done` (when kick sequence has finished)

## Dependencies

- Director
