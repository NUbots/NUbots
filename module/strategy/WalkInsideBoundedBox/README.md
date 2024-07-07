# WalkInsideBoundedBox

## Description

Keeps the robot clamped within bounded box area on the field. The bounded box is configurable.

If ball is outside bounded box area and ball is in front of the bounding box, defender is clamped to the edge of bounding box closest to ball.

If ball is outside the bounded box area and to the side of the bounding box, the robot is clamped to the edge of bounding box closest to ball and positions itself 1 meter behind the ball.

## Usage

Add this module to keep the robot within a bounding box on the field.

## Consumes

- `message::input::Sensors` To get sensor information for localisation
- `message::localisation::Field` Localisation usage for keeping inside bounded box
- `message::localisation::FilteredBall` Used for ball position
- `message::strategy::WalkInsideBoundedBox` a Task requesting to stay within a bounding box on the field

## Emits

- `message::strategy::WalkToFieldPosition` a Task to walk to desired field position

## Dependencies
