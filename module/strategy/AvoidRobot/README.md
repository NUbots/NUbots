# AvoidRobot

## Description

`AvoidRobot` monitors nearby robots and issues short-term walking proposals to sidestep or retreat to avoid collisions.

It selects the nearest detected robot (in robot-frame coordinates) and decides whether to enter or exit avoidance using two guards:

- A near-field override that forces avoidance when another robot is very close.
- A front-gate plus distance threshold (with hysteresis) that normally activates avoidance only for obstacles ahead of the robot.

When active, the module blends a deterministic lateral sidestep with a small retreat component, normalises the result, and emits a `message::planning::WalkProposal` to steer the robot away.

## Usage

Add this module and emit the `message::strategy::AvoidRobot` Task to request avoidance behaviour.

## Consumes

- `message::localisation::Robots` for nearby robot positions
- `message::input::Sensors` for `Hrw` transform into robot frame

## Emits

- `message::planning::WalkProposal` when avoidance is active

## Dependencies

- Uses `Sensors.Hrw` to transform opponent positions into robot frame
