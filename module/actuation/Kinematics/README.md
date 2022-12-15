# Kinematics

## Description

Provides inverse kinematics for left leg, right leg or head. Will emit a Done Task when the corresponding limb is Done.

## Usage

Emit the corresponding IK message as a task.

## Consumes

- `message::actuation::LeftLegIK` with information on the servo gains, torque, time to reach the position and the matrix from the target position for the left foot to the torso.
- `message::actuation::RightLegIK` with information on the servo gains, torque, time to reach the position and the matrix from the target position for the right foot to the torso.
- `message::actuation::HeadIK` with information on the servo gains, torque, time to reach the position and the vector that the head should look.

## Emits

- `message::actuation::LeftLeg` with servo control information for each servo in the left leg.
- `message::actuation::RightLeg` with servo control information for each servo in the right leg.
- `message::actuation::Head` with servo control information for each servo in the head.
- `Done` when the limb Tasks are Done.

## Dependencies

- `utility/actuation/InverseKinematics.hpp` provides the IK functions
