# AlignRobotToBall

## Description

Rotates on the spot until the robot is facing the ball

## Usage

Add this module to align the robot to the ball

## Consumes

- `message::strategy::AlignRobotToBall` Task requesting to rotate towards the ball
- `message::Localisation::Ball` Information on where the ball is
- `message::input::Sensors` Information regarding the robots odometry

## Emits

- `message::planning::TurnOnSpot` Task requesting robot to rotate on the spot

## Dependencies

- Eigen
