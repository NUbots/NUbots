# WalkToBall

## Description

Walks to the field position using localisation information.

## Usage

Add this module to walk to position on the field.

## Consumes

- `message::strategy::WalkToFieldPosition` Task requesting to walk to position on field
- `message::localisation::FilteredBall` information on where the ball is

## Emits

- `message::planning::WalkTo` Task requesting to walk to a location, in this case the ball

## Dependencies
