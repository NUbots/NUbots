HeadController
=============

## Description

Controls the motion of the head

## Usage

Takes a HeadCommand message:

- If command requests smoothing (`command->smooth = true`), then the goal angle emitted to is smoothed using exponential filter
- If command request is in world space (`command->goalRobotSpace = false`) the goal angle is converted from world space to robot space

The final goal angle is clamped based on the maximum/minium pitch and yaw of the head servos before being sent to servo controller

## Consumes

* `message::motion::HeadCommand` command containing desired angles, smoothing flag and reference frame information

## Emits

*`message::behaviour::ServoCommands` instructs servo controller to move head to desired angle

## Dependencies
