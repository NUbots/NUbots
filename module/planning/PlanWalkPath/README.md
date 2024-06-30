# PlanWalkPath

## Description

Plans a walk path based on information given. Three types of walk path planning are in this module

1. Walking towards a particular point. This uses a point in robot space and walks to it, aligning with the direction to face at the target and avoiding robots.
   It avoids robots by finding the first robot that intersects with the path, grouping it with any close by robots, then walking to a spot to the left or right of the group.
   Once its cleared the group, it continues on to either avoid another robot or wakl straight to the target
2. Turn on the spot. Without moving translationally, turn on the spot. Supports both directions.
3. Rotate around the ball. Rotate such that the ball stays in front of the robot, and the robot turns such that it changes the direction it is aligned towards. Supports both directions.

## Usage

Include this module in the role and emit the provide Task for either reaction. The reactions cannot run at the same time, since they both Need the Walk.

## Consumes

- `message::planning::WalkTo` Task requesting to walk towards a given point.
- `message::planning::TurnOnSpot` Task requesting to turn on the spot.
- `message::planning::PivotAroundPoint` Task requesting to rotate around the ball, to align with a direction.
- `message::localisation::Robots` to find any robots to avoid when path planning to a target.
- `message::input::Sensors` to transform robot points into (our own) robot space.

## Emits

- `message::skill::Walk` Task requesting to run the walk engine, giving the xy velocity (m/s) and rotational velocity (radians/s).

## Dependencies

- The walk engine
- Mathematics comparison utility
- Mathematics intersection utility
