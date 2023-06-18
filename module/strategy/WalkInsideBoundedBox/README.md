# WalkInsideBoundedBox

## Description

Keeps the robot clamped inside a bounded box, from which it will play soccer normally.

If ball is outside bounded box area and ball is in opposition half, defender is clamped to the edge of bounding box closest to ball.

If ball is outside bounded box area and ball is in own half, robot is clamped to the edge of bounding box closest to ball and positions itself 1 meter behind the ball.

## Usage

Add this module to play inside a bounded box in soccer!

## Consumes

- `message::input::Sensors` To get raw sensor information for localisation
- `message::localisation::Field` Localisation usage for keeping inside bounded box

## Emits

- `message::strategy::WalkToFieldPosition` a task to walk to desired field position

## Dependencies
