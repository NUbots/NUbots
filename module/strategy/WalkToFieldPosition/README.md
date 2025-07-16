# WalkToFieldPosition

## Description

Walks to the field position using localisation information with time-based hysteresis to prevent oscillations due to localization drift.

## Usage

Add this module to walk to a position on the field.

## Features

### Hysteresis Mechanisms

The module includes time-based hysteresis to prevent the robot from stopping and starting due to localization drift:

1. **Basic Hysteresis**: Uses different thresholds for stopping (`stop_threshold`) and resuming (`stopped_threshold`)
2. **Time-based Hysteresis**: Requires the robot to be within/outside the threshold for a minimum time before stopping/resuming

### Configuration Parameters

- `stop_threshold`: Error threshold for stopping the robot (default: 0.15m)
- `stopped_threshold`: Error threshold for resuming walking (default: 0.4m)
- `min_stop_time`: Minimum time robot must be within threshold before stopping (default: 0.5s)
- `min_resume_time`: Minimum time robot must be outside threshold before resuming (default: 1.0s)

## Consumes

- `message::strategy::WalkToFieldPosition` Task requesting to walk to position on field
- `message::localisation::Field` Field localization information
- `message::input::Sensors` Robot sensor data

## Emits

- `message::planning::WalkTo` Task requesting to walk to a point
- `message::skill::Walk` Task to stop the robot when target is reached

## Dependencies
