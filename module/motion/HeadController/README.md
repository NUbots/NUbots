# HeadController

## Description

Controls the motion of the head

## Usage

Takes a HeadCommand message:

- If command requests smoothing (`command->smooth = true`), then the goal angle emitted to is smoothed using exponential filter, tunable using `smoothing_factor`
- If command request is in world space (`command->goalRobotSpace = false`) the goal angle is converted from world space to robot space

The final goal angle is clamped based on the maximum/minium pitch and yaw of the head servos before being sent to servo controller

## Consumes

- `message::motion::HeadCommand` command containing desired angles, smoothing flag and reference frame information
- `message::input::Sensors` contains information to map desired angles into robot frame from world frame
- `message::actuation::KinematicsModel` contains maximum and minimum values for head pitch/yaw

## Emits

- `message::motion::HeadCommand` instructs servo controller to move head to initial position when configuring
- `message::behaviour::ServoCommands` instructs servo controller to move head to desired goal angle
- `utility::behaviour::RegisterAction` registers the module so it is allowed to move the servos. It registers for access to the head with a priority of 30 (a higher number is a higher priority). If it loses priority of the head, it disables the main reaction loop and if it gets back priority it enables the reaction again.

## Dependencies

- `Eigen`
- `InverseKinematics`
- `utility/math/comparison.hpp` for clamping
- `utility/math/coordinates.hpp` for converting angles to vectors
