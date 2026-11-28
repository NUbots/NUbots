# Sensors

## Description

Emits a `message::input::Sensors` message based on input from the K1's odometry and sensor data.

## Usage

When installed, it subscribes to the head pose DDS topic at startup and reacts to incoming
`message::platform::RawSensors` to build and emit a
`message::input::Sensors` each tick along with the odometry from the K1. Button edge transitions are also detected and emitted as separate messages.

Odometry is reported relative to the pose at the last re-zero, which happens at startup, on
`ResetFieldLocalisation` and on any Booster mode change. HardwareIO also resets the controller's own odometry at
those times, but that lands asynchronously, so any jump in the raw odometry larger than `odometry_jump` is absorbed
into the zero offset instead of being reported: `Htw` only ever jumps at a re-zero. Twists faster than
`odometry_twist_max_speed` are discarded (`vTw` is NaN). A twist faster than `stuck_twist.speed` while the
odometry pose is stationary is reported as zero, since the controller's twist can latch a non-zero value for tens
of seconds while the robot stands.

The head pose reader is also created once at startup; changing `head_pose.topic` in config after that requires a
restart to take effect.

## Consumes

- `message::platform::RawSensors` to build the outgoing `Sensors` message, including servo, IMU, button, and LED
  state.
- `message::booster::BoosterOdometry`, paired with `RawSensors`, for the robot's odometry-derived world pose.
- `message::booster::BoosterOdometryTwist` (optional) for `vTw`, the body-frame twist rotated into the world by
  `Htw`. Without a twist newer than `odometry_twist_max_age`, `vTw` is NaN.
- `message::booster::BoosterModeState` to detect motion mode changes and re-zero the odometry.
- `message::localisation::ResetFieldLocalisation` to re-zero the odometry after a localisation reset.

## Emits

- `message::input::Sensors` with filtered/converted sensor data and odometry.
- `message::input::ButtonLeftDown` / `ButtonLeftUp` when the left button changes state.
- `message::input::ButtonMiddleDown` / `ButtonMiddleUp` when the middle button changes state.

## Dependencies

- Eigen
- Booster Robotics SDK
