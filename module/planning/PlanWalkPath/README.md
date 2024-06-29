# PlanWalkPath

## Description

Plans a walk path based on information given. Three types of walk path planning are in this module

1. Walking towards a particular point. This uses a point in torso space and walks directly to it, ignoring obstacles.
2. Turn on the spot. Without moving translationally, turn on the spot. Supports both directions.
3. Rotate around the ball. Rotate such that the ball stays in front of the robot, and the robot turns such that it changes the direction it is aligned towards. Supports both directions.

## Usage

Include this module in the role and emit the provide Task for either reaction. The reactions cannot run at the same time, since they both Need the Walk.

## Consumes

- `message::planning::WalkTo` Task requesting to walk towards a given point.
- `message::planning::TurnOnSpot` Task requesting to turn on the spot.
- `message::planning::PivotAroundPoint` Task requesting to rotate around the ball, to align with a direction.

## Emits

- `message::skill::Walk` Task requesting to run the walk engine, giving the xy velocity (m/s) and rotational velocity (radians/s).

## Dependencies

- The walk engine
- Mathematics comparison utility
