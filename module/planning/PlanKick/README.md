# PlanKick

## Description

The module plans when to kick the ball, using information about the position of the ball relative to the robot and where the robot would like to kick to. It only uses recent ball measurements.

It checks

1. If the robot is close enough to the ball.
2. If the robot is facing the ball.
3. If the robot is facing the target to kick to.

If these checks pass, it will emit a Kick Task.

There are configuration values that allow some flexibility with this method.

1. The third check can be skipped with the `align` configuration value.

## Usage

Emit a KickTo Task with the position to kick to. The contents of the message (position to kick to) is not important if the `align` configuration value is set to `false`. The reaction requires localisation ball measurements.

## Consumes

- `message::planning::KickTo` a signal to kick the ball when conditions are met, with the location to kick the ball to in the message.
- `message::localisation::Ball` for information on where the ball is.
- `message::input::Sensors` for the robot's pose, used to compute robot-relative ball position and kick direction.
- `message::localisation::Field` for the robot's field-space pose, used to compute the kick direction.
- `message::support::FieldDescription` (at startup) to compute the field-space goal target position (offset shared with WalkToBall's config).

## Emits

- `message::skill::Kick` a signal to execute a kick movement: `target` (the field-space point behind the goal line) and `direction` (a robot-relative unit vector from the ball toward that target).

## Dependencies

- Director
- The reaction will not run without localisation ball messages.
