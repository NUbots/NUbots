# PlanWalkPath

## Description

Plans a walk path based on information given. Two types of walk path planning are in this module

1. Walking to a particular point. This uses a point in torso space and walks directly to it, ignoring obstacles.
2. Pivoting. This could involve pivoting on the spot, or pivoting around an object in front of the robot (i.e. the ball). Pivoting can be done in either direction.

## Usage

Include this module in the role and emit the provide Task for either reaction. The reactions cannot run at the same time, since they both Need the Walk.

## Consumes

- `message::planning::WalkTo` Task requesting to walk to the given point.
- `message::planning::Pivot` Task requesting to pivot around the given point.

## Emits

- `message::skill::Walk` Task requesting to run the walk engine, giving the xy velocity (m/s) and rotational velocity (radians/s).

## Dependencies
