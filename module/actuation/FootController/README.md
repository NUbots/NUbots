# FootController

## Description

Skill which corrects the foot orientation to level it with the ground. The correction of the foot can be disabled in the
config and the module will request the given orientation without correction.

## Usage

Include this module to allow the robot to control the feet of robot.

## Consumes

- `message::actuation::ControlLeftFoot` Task requesting the left leg is moved to desired pose.
- `message::actuation::ControlRightFoot` Task requesting the right leg is moved to desired pose.

## Emits

- `message::actuation::LeftLegIK` Task requesting the left leg is moved using Inverse Kinematics, containing left leg motion information.
- `message::actuation::RightLegIK` Task requesting the right leg is moved using Inverse Kinematics, containing right leg motion information.

## Dependencies

- Eigen
