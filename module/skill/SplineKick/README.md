# SplineKick

## Description

SplineKick module executes a kicking motion using splines for smooth and accurate movement. Waypoints for the kicking
foot and torso relative to the support foot are specified in the config and executed by this module.

## Usage

Add this module to allow the robot to kick.

## Consumes

- `message::skill::Kick` Task requesting the robot to kick

## Emits

- `message::actuation::ControlLeftFoot` Task requesting the left leg is moved to desired pose.
- `message::actuation::ControlRightFoot` Task requesting the right leg is moved to desired pose.
- `message::actuation::LeftArm` Task requesting the left arm is moved using Inverse Kinematics, containing left arm servo commands.
- `message::actuation::RightArm` Task requesting the right arm is moved using Inverse Kinematics, containing right arm servo commands.

## Dependencies

- Eigen
