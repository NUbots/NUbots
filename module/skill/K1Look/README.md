# K1Look

## Description

Converts head look-at commands into head rotation commands for the Booster Robotics SDK, to move the K1's
head using its built-in head control.

Takes the look direction from a `message::skill::Look` task, optionally smooths it with an exponential
filter, converts it into head yaw/pitch angles, and emits a `message::booster::BoosterHeadRot` for the
Booster HardwareIO to forward to the SDK.

## Usage

Include this module to allow the robot to look in a given direction on the K1. Use in place of
`skill::Look`, which instead drives the head via the NUbots servo/kinematics stack.

## Consumes

- `message::skill::Look` A Task requesting to look in a direction, containing the vector `rPCt` (camera to
  target, in torso space) and a `smooth` flag.

## Emits

- `message::booster::BoosterHeadRot` containing the desired head yaw/pitch rotation, consumed by the
  Booster HardwareIO.

## Dependencies

- Director
- Eigen
