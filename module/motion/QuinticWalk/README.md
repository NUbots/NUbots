# QuinticWalk

## Description

Open loop walk engine that uses quintic splines to create trajectories. [Created by Bit-Bots.](https://github.com/bit-bots/bitbots_motion). Based off code by Quentin "Leph" Rouxel and Team Rhoban.

## Usage

- Send a `WalkCommand` of the form (x meters/second, y meters/second, radians/second) to give the QuinticWalk a direction to go in.

- Use `EnableWalkEngineCommand` to enable the walk.

- Use `DisableWalkEngineCommand` to disable the walk.

- Use `StopCommand` to stop the walk.

- Will stop when `ExecuteGetup` is sent

- Will be able to walk again during get up if it receives a `KillGetup`

- Config values can be changed in `QuinticWalk.yaml`.

## Emits

`message::behaviour::ServoCommand`

## Dependencies

- Eigen

- KinematicsModel

- Sensors

- InverseKinematics

- `utility::motion::splines::*`
