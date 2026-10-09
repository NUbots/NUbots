# K1Sensors

## Description

Emits a `message::input::Sensors` message based on input from the K1's odometry and sensor data.

## Usage

When installed, it subscribes to the head pose DDS topic at startup and reacts to incoming
`message::platform::RawSensors` to build and emit a
`message::input::Sensors` each tick along with the odometry from the K1. Button edge transitions are also detected and emitted as separate messages.

The head pose reader is also created once at startup; changing `head_pose.topic` in config after that requires a
restart to take effect.

## Consumes

- `message::platform::RawSensors` to build the outgoing `Sensors` message, including servo, IMU, button, and LED
  state.
- `message::booster::BoosterOdometry`, paired with `RawSensors`, for the robot's odometry-derived world pose.
- `message::booster::BoosterModeState` to detect motion mode changes and reset the odometry zero offset.
- `message::localisation::ResetFieldLocalisation` to reset the odometry zero offset after a localisation reset.

## Emits

- `message::input::Sensors` with filtered/converted sensor data and odometry.
- `message::input::ButtonLeftDown` / `ButtonLeftUp` when the left button changes state.
- `message::input::ButtonMiddleDown` / `ButtonMiddleUp` when the middle button changes state.

## Dependencies

- Eigen
- Booster Robotics SDK
