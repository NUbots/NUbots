# Getup

## Description

A Behaviour module that will run the appropriate getup script for the robot. First a StandStill task is emitted, then the getup task is delayed so that the robot can settle into it's final position to improve side detection accuracy. Additionally, if after the delay the robot detects that it is on its left/right side, another delay will be emitted.

## Usage

Include this module to allow the robot to get up after it has fallen over.

## Consumes

- `message::skill::GetUp` as a Task to trigger execution
- `message::input::Sensors` to determine which get up script to execute

## Emits

- `message::behaviour::state::Stability` to update the stability when starting and finishing getting up
- `message::actuation::BodySequence` as a Task containing the script sequence for getting up

## Dependencies

- The `Script` extension to convert scripts into BodySequences
