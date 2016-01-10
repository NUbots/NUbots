Darwin Sensor Filter
=====================

## Description

Uses Kalman filters in order to filter the raw incoming data, and provide clean platform agnostic data.

## Usage

When installed, it will read incoming `message::platform::darwin::DarwinSensorData` objects and pass them through relevant kalman filters.
The resulting filtered data will then be outputted as `message::input::Sensors` to be used by the rest of the system.

## Consumes

* `message::DarwinSensors` in order to filter them.

## Emits

* `message::input::Sensors` with filtered data from the input.
* 
## Dependencies

* The Darwin Hardware I/O module is required to provide the data to filter

