# PlanKick

## Description

The module plans when to kick the ball, using information about the position of the ball relative to the robot and where the robot would like to kick to. It only uses recent ball measurements.

It checks

1. If the robot is close enough to the ball.
2. If the robot is facing the ball.
3. If the robot is facing the target to kick to.

If these checks pass, it will emit a Kick Task, specifying the leg to kick with based on which foot the ball is closer to.

There are configuration values that allow some flexibility with this method.

1. The third check can be skipped with the `align` configuration value.
2. A particular kick leg can be forced instead of checking the position using the `kick_leg` configuration value.

## Usage

Emit a KickTo Task with the position to kick to. The contents of the message (position to kick to) is not important if the `align` configuration value is set to `false`. The reaction requires localisation ball measurements.

## Consumes

- `message::planning::KickTo` a signal to kick the ball when conditions are met, with the location to kick the ball to in the message.
- `message::localisation::Ball` for information on where the ball is.

## Emits

- `message::skill::Kick` a signal to execute a kick movement, containing information on what leg to kick with.

## Dependencies

- The reaction will not run without localisation ball messages.
