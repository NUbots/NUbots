# Odometry Data Collector

## Description
This module collects raw kinematic and control inputs along with ground-truth simulator poses to generate odometry training datasets for neural networks (e.g. AutoOdom).

## Usage
Add this module to your role alongside `support::logging::DataLogging` and an active simulation environment (e.g. Webots). Drive the robot manually to collect varying gaits and displacement data.

## Emits
- `message::localisation::OdometryRecord`: A time-synchronised record of inputs and expected outputs (displacements) for odometry learning.

## Dependencies
- `message::input::Sensors`
- `message::behaviour::state::WalkState`
- `message::localisation::RobotPoseGroundTruth`
