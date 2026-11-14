# Look

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
  target, in torso space), a `smooth` flag and a `hold_yaw` flag. With `hold_yaw`, the direction's yaw is held in the
  world from when the Look is given: the head turns against the robot's turning, up to `max_hold_yaw`.
- `message::input::Sensors` for the robot's yaw, when a Look has `hold_yaw`.

## Emits

- `message::booster::BoosterHeadRot` containing the desired head yaw/pitch rotation, consumed by the
  Booster HardwareIO.

## Dependencies

- Director
- Eigen
