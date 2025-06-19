# Defend

A defending orientated robot.

## Description

The robot hangs back in line with the edge of the penalty box, in line with vector between our own goal and the ball.

## Usage

Designed for keeping the robot at the back in case the ball is taken towards our goal. If the ball gets close, the robot should switch to an attacking robot to take the ball.

## Consumes

- `message::purpose::Defend` a task telling the robot to be a defending player.
- `message::localisation::Ball` to identify where to position relative to the ball.
- `message::localisation::Field` to measure the position to walk to in field space.
- `message::support::FieldDescription` to measure where the goals and penalty box are.

## Emits

- `message::strategy::WalkToFieldPosition` to tell the robot where to walk to on the field.

## Dependencies

- `utility::math::euler::pos_rpy_to_transform` to change the position and orientation into a homogenous transformation matrix.
