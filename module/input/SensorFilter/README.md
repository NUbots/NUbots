# Sensor Filter

## Description

Uses a Unscented Kalman filter to filter the raw incoming data, and provide clean platform agnostic data.

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
- `message::support::nusight::DataPoint` if log_level >= DEBUG, indicating if feet are down.

## Dependencies
