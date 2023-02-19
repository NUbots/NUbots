# QuinticWalk

## Description

Open loop walk engine that uses quintic splines to create trajectories.

## Usage

Include this module to allow the robot to walk.

## Consumes

- `message::skill::Walk` containing a vector with the desired velocity target.

## Emits

- `message::behaviour::state::Stability` to update the system's stability state based on the walk engine state.
- `message::actuation::LeftLegIK` containing left leg motion information.
- `message::actuation::RightLegIK` containing right leg motion information.
- `message::actuation::LeftArm` containing left arm servo commands.
- `message::actuation::RightArm` containing right arm servo commands.

## Dependencies

- Eigen
- KinematicsModel
- Sensors
- `utility::motion::splines::*`
