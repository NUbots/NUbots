# Sensors

## Description

Emits a `message::input::Sensors` message based on input from the K1's odometry and sensor data.

## Usage

When installed, it subscribes to the head pose DDS topic at startup and reacts to incoming
`message::platform::RawSensors` to build and emit a
`message::input::Sensors` each tick along with the odometry from the K1. Button edge transitions are also detected and emitted as separate messages.

Odometry is reported relative to the pose at the first sample and is never reset, so `Htw` is continuous. Twists
faster than `odometry_twist_max_speed` are discarded (`vTw` is NaN). A twist faster than `stuck_twist.speed` is
reported as zero while no leg joint has moved more than `stuck_twist.stationary_joint_range` over
`stuck_twist.stationary_window`, since the controller's twist can latch a non-zero value while the robot stands.

The head pose reader is also created once at startup; changing `head_pose.topic` in config after that requires a
restart to take effect.

## Consumes

- `message::platform::RawSensors` to build the outgoing `Sensors` message, including servo, IMU, button, and LED
  state. Its leg joint positions detect standing still.
- `message::booster::BoosterOdometry`, paired with `RawSensors`, for the robot's odometry-derived world pose.
- `message::booster::BoosterOdometryTwist` (optional) for `vTw`, the body-frame twist rotated into the world by
  `Htw`. Without a twist newer than `odometry_twist_max_age`, `vTw` is NaN.

## Emits

- `message::input::Sensors` with filtered/converted sensor data and odometry.
- `message::input::ButtonLeftDown` / `ButtonLeftUp` when the left button changes state.
- `message::input::ButtonMiddleDown` / `ButtonMiddleUp` when the middle button changes state.

## Dependencies

- Eigen
- Booster Robotics SDK
