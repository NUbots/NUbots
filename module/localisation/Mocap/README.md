# Mocap

## Description

The Mocap module processes motion capture data to provide ground truth robot pose information. It receives motion capture data containing rigid body positions and orientations, filters for the specific robot rigid body, and converts this data into a robot pose ground truth message.

## Usage

The module requires configuration through `Mocap.yaml` with the following parameters:

- `robot_rigid_body_id`: The ID of the rigid body that represents the robot in the motion capture system

Enable ground truth localisation in `SensorFilter` and `FieldLocalisationNLopt` module to use this data directly for pose estimation (odometry/localisation)

## Consumes

- `message::input::MotionCapture`: Motion capture data containing positions and orientations of tracked rigid bodies
- `extension::Configuration`: Configuration data from Mocap.yaml

## Emits

- `message::localisation::RobotPoseGroundTruth`: The ground truth pose of the robot, containing:
  - Position (x, y, z) in meters
  - Orientation as a quaternion (w, x, y, z)
  - Full transformation matrix (Hft) combining position and orientation

## Dependencies
