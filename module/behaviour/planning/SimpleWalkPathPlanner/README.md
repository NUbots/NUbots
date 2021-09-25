# Simple Walk Path Planner

## Description

Plans a path trajectory to the ball based on the latest recieved message. The pattern of the path followed will depend on the role of the robot and should be entirely contained within a function called from the switch case in SimpleWalkPathPlanner.cpp

## Usage

Include this module to allow the robot to plan a walk command for a walk strategy based on the role it is supposed to play on the field and to increase its efficiency in time spent navigating to the ball of a given position.

## Consumes

- `message::behaviour::MotionCommand` containing the walk type to use at this point in time
- `message::localisation::*` containing navigation data
- `message::vision::Balls` containing the estimated positions of a ball on the field from VisualMesh

## Emits

- `message::notyetimplemented` instructs the walk reflex to do a walk

## Dependencies

see consumes
