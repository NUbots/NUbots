# Getup

## Description

A Behaviour module that will run the appropriate getup script for the

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
