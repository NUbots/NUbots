# StaticWalk

## Description

This walk engine moves the robot's torso so that its center of mass is over its support foot. Then it tells the swing foot to move to a target in front so as to move at the speed given by the walk command. It then repeats these steps with the old swing foot as the new support foot.

## Usage

Runs with a `WalkCommand`, which can be received from walk path planner modules or direct controller modules such as `KeyboardWalk`.

Currently does not prevent the feet from colliding, particularly in cases with rotation and strafing.

## Consumes

- `message::motion::WalkCommand`, the command describing how fast and in what direction to walk. Gives a vector with three values, the first is the velocity in the x-axis in metres/second, the second is the velocity of the y-axis in metres/second, the third is the rotation in radians/second.
- `message::input::Sensors` to get information about the current state of the robot, including odometry and forward kinematics.
- `message::motion::EnableWalkEngineCommand` tells the walk engine it has just been re-enabled and resets it.
- `message::motion::DisableWalkEngineCommand` disables the walk engine reactor, stopping it from running when not needed.
- `message::motion::StopCommand` sets the walk command to zero, keeping the walk engine running but not moving the robot.

## Emits

- `message::motion::TorsoTarget`, the target for the torso to move to which will be handled by the MovementController
- `message::motion::FootTarget`, the target for the swing foot to move to which will be handled by the MovementController

## Dependencies

- [Eigen Linear Algebra Library](https://eigen.tuxfamily.org/index.php)
