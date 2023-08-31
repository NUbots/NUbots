# Walk

## Description

Open loop walk engine that uses quintic splines to create swing foot and torso trajectories.

## Usage

Include this module to allow the robot to walk.

## Consumes

- `message::skill::Walk` A Task requesting to walk, containing a vector with the desired velocity target.

## Emits

- `message::behaviour::state::Stability` to indicate when the robot is walking (dynamically stable) and standing.
- `message::actuation::ControlLeftFoot` Task requesting the left leg is moved to desired pose.
- `message::actuation::ControlRightFoot` Task requesting the right leg is moved to desired pose.
- `message::actuation::LeftArm` Task requesting the left arm is moved using Inverse Kinematics, containing left arm servo commands.
- `message::actuation::RightArm` Task requesting the right arm is moved using Inverse Kinematics, containing right arm servo commands.

## Dependencies

- Eigen
