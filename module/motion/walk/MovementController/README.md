# MovementController

## Description

Dynamically moves the robot's feet along a trajectory defined by a vector field to reach a given target location.

## Usage

## Consumes

- `message::input::Sensors` to get information about the current state of the robot, including odometry and forward kinematics.
- `message::motion::KinematicsModel` to get information on the structure of the robot for bounds checking and calculating inverse kinematics.
- `message::motion::FootTarget` to get information on where and when to move a particular foot to.
- `message::motion::TorsoTarget` to get information on when and where to move the torso to.

## Emits

- `message::behaviour::ServoCommands` which tell the servos what joint angle to move to and when to reach that position by.

## Dependencies

- [Eigen Linear Algebra Library](https://eigen.tuxfamily.org/index.php)
