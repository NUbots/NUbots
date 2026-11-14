# NUSimGroundTruth

## Description

Reads the ground truth pose of the robot and the ball published by **NUSim** over DDS and, while `ground_truth` is
enabled, emits `message::localisation::Field` and `message::localisation::Ball` directly from it. This lets behaviour
be tested in the simulator without vision or localisation in the loop.

While ground truth is active the module emits `message::localisation::GroundTruthMode` so that the vision-based
localisation modules (`localisation::FieldLocalisationNLopt` and `localisation::BallLocalisation`) stop emitting their
own estimates. If no ground truth arrives for longer than `timeout`, ground truth mode is switched off again and vision
and localisation take over.

Use this module in `roles/nusim/*` only.

## Usage

`data/config/NUSimGroundTruth.yaml` contains:

- `ground_truth`: whether to use NUSim's ground truth in place of vision and localisation. NUSim's match launcher sets
  this per robot when "Perception: Ground truth" is chosen
- `robot_topic`: DDS topic for the robot's ground truth pose (`nav_msgs/Odometry` in NUSim's world frame). Only read at
  startup
- `ball_topic`: DDS topic for the ball's ground truth position and velocity (`nav_msgs/Odometry` in NUSim's world
  frame). Only read at startup
- `world_to_field_yaw`: yaw from NUSim's world frame to this robot's field frame {f}, which has our own goal on +x
- `timeout`: ground truth older than this (in seconds) is treated as missing

## Consumes

- `message::input::Sensors` uses the odometry transform from world {w} to torso {t} to compute where the field is in
  world space, and the camera transform for the ball message

## Emits

- `message::localisation::GroundTruthMode` whenever ground truth perception switches on or off
- `message::localisation::RobotPoseGroundTruth` the ground truth transform from torso {t} to field {f} space
- `message::localisation::Field` the ground truth transform from world {w} to field {f} space
- `message::localisation::Ball` the ground truth ball position and velocity in world {w} space

## Dependencies

- `Eigen`
- Booster Robotics SDK for the DDS channels
- NUSim publishing the ground truth topics (see NUSim's `SdkBridge/PROTOCOL.md`)
