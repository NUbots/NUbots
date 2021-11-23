# MovementController

## Description

Dynamically moves the robot's feet along a trajectory defined by a vector field to reach a given target location at a particular time.

The swing foot is guided along a vector field in the x-z plane representing a step. Any change in the y-axis is calculated by linear interpolation. The rotation of the foot is changed using spherical linear interpolation (slerp). The torso is moved to its position using linear interpolation.

Calculations are relative to ground space, which has its origin on the support foot and rotation equal to the world. That is, ground space is the support foot if it were flat on the ground.

Contains

- `FootController` which calculates the next position for the swing foot using the vector field.
- `TorsoController` which calculates the next position of the torso using linear interpolation.
- `MovementController` which handles the input of the `FootTarget` and `TorsoTarget`, uses the `FootController` and `TorsoController` to get the next positions of the feet, and emits the `ServoCommands`.

## Usage

The MovementController needs both a `FootTarget` and a `TorsoTarget` to run. It's designed to be used with a walk engine that sends `FootTarget` messages for the swing foot and `TorsoTarget` messages for the support foot/torso. The swing foot is the foot that is moving to take a step and the support foot is the foot in contact with the ground while the swing foot steps. If the swing foot needs to stay on the ground, e.g. while a static walk engine is leaning over the support foot and doesn't want to step, this is handled with the `FootTarget`'s `Mode` value by setting it to `NO_LIFT`.

Homogeneous transformation matrices that define the position of the end of the foot relative to the torso are calculated for both feet. It uses the InverseKinematics utility to find joint positions for these calculated transformation matrices. These joint positions are then emitted as `ServoCommands`.

## Consumes

- `message::input::Sensors` to get information about the current state of the robot, including odometry and forward kinematics.
- `message::motion::KinematicsModel` to get information on the structure of the robot for bounds checking and calculating inverse kinematics.
- `message::motion::FootTarget` to get information on where and when to move a particular foot to.
- `message::motion::TorsoTarget` to get information on when and where to move the torso to.

## Emits

- `message::behaviour::ServoCommands` which tell the servos what joint angle to move to and when to reach that position by.

## Dependencies

- [Eigen Linear Algebra Library](https://eigen.tuxfamily.org/index.php)
- `utility::motion::kinematics::calculateLegJoints`, a utility to calculate the joint positions of a leg (or legs) given a homogeneous transformation matrix of the desired target position of the foot relative to the torso.
- `utility::motion::walk`, utilities for the vector field for the `FootController`.
- `utility::motion::kinematics::calculateGroundSpace`, calculates are done relative to ground space, which is retrieved from this utility. Ground space has its origin on the support foot and rotation equal to the world. That is, ground space is the support foot if it were flat on the ground.
