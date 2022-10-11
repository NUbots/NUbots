Kinematics
==========

## Description

Provides inverse kinematics for left leg, right leg or head. Will emit a done task when the current time is after the time the servo should reach its position.

## Usage

Emit the corresponding IK message as a task.

## Consumes

- `message::motion::LeftLegIK` with information on the servo gains, torque, time to reach the position and the matrix from the target position for the left foot to the torso.
- `message::motion::RightLegIK` with information on the servo gains, torque, time to reach the position and the matrix from the target position for the right foot to the torso.
- `message::motion::HeadIK` with information on the servo gains, torque, time to reach the position and the vector that the head should look.
- `message::input::Sensors` to check when the task is `Done`.

## Emits

- `message::motion::LeftLeg` with servo control information for each servo in the left leg.
- `message::motion::RightLeg` with servo control information for each servo in the right leg.
- `message::motion::Head` with servo control information for each servo in the head.
- `Done` when the servos have moved according to the IK message (`LeftLegIK`, `RightLegIK`, `HeadIK`).

## Dependencies

- `utility/motion/InverseKinematics.hpp`
