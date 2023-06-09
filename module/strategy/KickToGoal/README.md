# KickToGoal

## Description

Kicks towards the goals.

## Usage

Add this module to kick towards the goals.

## Consumes

- `message::strategy::KickToGoal` The Task requesting to kick towards the goal.
- `message::localisation::Field` Information on where the field is in the world.
- `message::input::Sensors` Information on where the robot is in the world.
- `message::support::FieldDescription` Information on the dimensions of the field, to get the goal position.

## Emits

- `message::planning::KickTo` a Task requesting to kick to the goals when applicable.

## Dependencies
