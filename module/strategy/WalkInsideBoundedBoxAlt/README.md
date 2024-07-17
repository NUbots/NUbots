# WalkInsideBoundedBox

## Description

Alternate Bounded box behaviour for Robocup2024

Keeps the robot clamped within bounded box area on the field. The bounded box is configurable.

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
