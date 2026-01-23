# Support

## Description

This module runs a support field player. It stays aligned with the ball on the y-axis, and stays a quarter of the field length away from the ball on the y-axis on whichever side is closest to the robot. It is then ready to take over as the attacker if the attacker becomes inactive.

## Usage

Include this module and emit a Support Director task to have the robot act as a support player. Intended to be used when teammates already occupy the attack and defend roles.

## Consumes

- `message::purpose::Support` a task telling the robot to be a supporting player.
- `message::localisation::Ball` information on where the ball is, to align with it.
- `message::localisation::Field` to perform calculations in field space for positioning.
- `message::input::Sensors` to get the position of the robot.
- `message::support::FieldDescription` for positioning relative to field width.

## Emits

- `message::strategy::WalkToFieldPosition` to tell the robot where to walk to on the field.

## Dependencies

- `utility::math::euler::pos_rpy_to_transform` to change the position and orientation into a homogenous transformation matrix.
