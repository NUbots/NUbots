# WalkInsideBoundedBox

## Description

Keeps the robot clamped within bounded box. The bounded box ios configurable.

If ball is outside bounded box area and ball is in opposition half, defender is clamped to the edge of bounding box closest to ball.

If ball is outside bounded box area and ball is outside bounded box and less than x coordinate, robot is clamped to the edge of bounding box closest to ball and positions itself 1 meter behind the ball.

## Usage

Add this module to play inside a bounded box in soccer!

## Consumes

- `message::input::Sensors` To get raw sensor information for localisation
- `message::localisation::Field` Localisation usage for keeping inside bounded box
- `message::localisation::FilteredBall` Used for ball position
- `message::purpose::Defender` To get defender tasks

## Emits

- `message::strategy::WalkToFieldPosition` a task to walk to desired field position

## Dependencies
