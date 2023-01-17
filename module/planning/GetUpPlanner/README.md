# GetUpPlanner

## Description

The GetUpPlanner module is used to automatically trigger a get up when the robot has fallen over.
It monitors the robot to see when it is lying down and has been relatively still for the past n frames before triggering a getup action.

## Usage

Emit a `message::plan::GetUpWhenFallen` as a Task.
Make sure that it has higher challenge priority than other recovery related tasks such as falling relax!
If relax has higher priority it may trigger mid getup

## Consumes

- `message::input::Sensors` to determine when the robot is in an appropriate state to get up
- `message::plan::GetUpWhenFallen` as a Task to activate this module to watch for when to run a getup

## Emits

- `message::skill::GetUp` as a task to trigger a get up action when appropriate

## Dependencies

- A module that can provide GetUp
