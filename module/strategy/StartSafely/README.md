# StartSafely

## Description

Use this module to make the robot move safely to a stand position.

## Usage

Emit the Task on Startup at the root of the Director tree.

## Consumes

- `message::strategy::StartSafely` Task to request the robot to start safely.
- `message::input::Sensors` to determine the position of the motors.

## Emits

- `message::actuation::Body` to move the whole body to the stand position.

## Dependencies
