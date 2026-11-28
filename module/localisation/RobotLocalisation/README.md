# RobotLocalisation

## Description

Estimates the position and velocity of other robots on the field.

The module works by tracking multiple robots using a UKF filter for each. Each vision detection gets its own measurement covariance, built from its range from the camera (range error grows with range squared, as it comes from ground-projecting the robot's feet). Detections are gated against each track by squared Mahalanobis distance (`gate_chi2`) and assigned one-to-one, lowest distance first; a detection that matches no track starts a new one. Messages over the network from other robots are used to determine whether a tracked robot is a teammate or opponent and to update the filter.

Opponent tracks are removed when they have had no vision update for `max_track_age` seconds, or when they are within `merge_distance` of an older track or a teammate. `max_robot_tracks` optionally limits the number of opponent tracks, keeping those with the most vision updates. Teammates are removed after being missed `max_missed_count` times in a row while they should be visible.

## Usage

Incluide this role to track other robots on the field.

## Consumes

- `message::vision::Robots` uses the robot position estimates from vision
- `message::input::RoboCup` uses teammate position from their WiFi message
- `message::input::Image` uses the FOV and image bounds from the camera information
- `message::localisation::Field` uses the field transformation matrix Hfw to get the location of the tracked robots in field space
- `message::support::FieldDescription` uses the field dimensions to determine whether the robot is outside the field (plus a given distance outside the field)
- `message::input::GameState` to get the robot's team colour for visualisation in NUsight

## Emits

- `message::localisation::Robots` contains filtered robot positions and velocity estimates
