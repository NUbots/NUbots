# RobotLocalisation

## Description
Estimates the position and velocity of other robots on the field.

The module works by tracking multiple robots using a UKF filter for each. Vision measurements are associated with each tracked robot using global nearest neighbor and an acceptance radius.

Tracked robots are discarded if they are not seen for a consecutive number of times when they should be visible.

## Usage

Incluide this role to track other robots on the field.

## Consumes

- `message::vision::Robots` uses the robot position estimates from vision
- `message::vision::GreenHorizon` uses the GreenHorizon to manage tracked robots

## Emits

- `message::localisation::Robots` contains filtered robot positions and velocity estimates

## Dependencies
