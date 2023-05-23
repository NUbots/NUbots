# Sensor Filter

## Description

Uses a Unscented Kalman filter to filter the raw incoming data, and provide clean platform agnostic data.

We assume that the IMU (accelerometer and gyroscope) are oriented to conform with the standard coordinate system x-axis out
the front of the robot y-axis to the left z-axis up

For the accelerometer the orientation should be as follows
x axis reports a +1g acceleration when robot is laying on its back
y axis reports a +1g acceleration when robot is laying on its right side
z axis reports a +1g acceleration when robot is vertical

## Usage

When installed, it will read incoming `message::platform::RawSensors` objects and pass them through the kalman filter.
The resulting filtered data will then be outputted as `message::input::Sensors` to be used by the rest of the system.

## Consumes

- `message::platform::RawSensors` in order to filter them.

## Emits

- `message::input::Sensors` with filtered data from the input.
- `message::platform::ButtonLeftDown` when button pressed.
- `message::platform::ButtonLeftUp` when button released.
- `message::platform::ButtonMiddleDown` when button pressed.
- `message::platform::ButtonMiddleUp` when button released.
- `message::eye::DataPoint` if log_level >= DEBUG, indicating if feet are down.

## Dependencies
