# MaintainBallDistance

## Description

Ensures the robot maintains a minimum distance from the ball. If the robot is already far enough away, it does nothing and lets the parent task drive positioning. If too close, it walks to a point `min_distance` metres from the ball along the ball-to-robot axis while facing the ball.

## Usage

Emit a `MaintainBallDistance` task with the desired `min_distance`. Typically emitted alongside a positioning task (e.g. `Defend` or `Support`) at a higher priority so it overrides the walk when the robot is too close, and yields when the distance is satisfied.

## Consumes

- `message::strategy::MaintainBallDistance` Task with the `min_distance` field specifying the minimum distance to maintain from the ball in metres
- `message::localisation::Ball` for the ball position in world space
- `message::localisation::Field` for the field-to-world transform
- `message::input::Sensors` for the robot-to-world transform

## Emits

- `message::strategy::WalkToFieldPosition` Task to walk to a position `min_distance` from the ball along the ball-to-robot axis, facing the ball

## Dependencies

- Director
- `utility::math::angle::vector_to_bearing` for computing the heading toward the ball
- `utility::math::euler::pos_rpy_to_transform` for constructing the target pose
