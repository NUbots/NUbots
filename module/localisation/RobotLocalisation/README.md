# RobotLocalisation

## Description

Estimates the position and velocity of other robots on the field.

The module works by tracking multiple robots using a UKF filter for each. Vision measurements are associated with each tracked robot using global nearest neighbor and an acceptance radius. Messages over the network from other robots are used to determine whether a tracked robot is a teammate or opponent and to update the filter.

Tracked robots are discarded if they are not seen for a consecutive number of times when they should be visible.

## Usage

Include this role to track other robots on the field.

## Consumes

- `message::vision::Robots` uses the robot position estimates from vision
- `message::input::RoboCup` uses teammate position from their WiFi message
- `message::vision::GreenHorizon` uses the GreenHorizon to manage tracked robots
- `message::localisation::Field` uses the field transformation matrix Hfw to get the location of the tracked robots in field space
- `message::support::FieldDescription` uses the field dimensions to determine whether the robot is outside the field (plus a given distance outside the field)
- `message::input::GameState` to get the robot's team colour for visualisation in NUsight

## Emits

- `message::localisation::Robots` contains filtered robot positions and velocity estimates

## Dependencies
