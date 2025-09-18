# KinematicsModel

## Description

Populates a KinematicsModelmessage and ServoOffsets message using values in the configuration files for the rest of the system to use throughout the program. Only emits once at the start.

## Usage

Include this module to use messages KinematicsModel and ServoOffsets

## Emits

- `message::actuation::KinematicsModel` describing the measurements of the physical robot model.
- `message::actuation::ServoOffsets` containing each servos unique offset to zero out the robot positions.

## Dependencies
