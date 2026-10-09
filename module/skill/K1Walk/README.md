# K1Walk

## Description

Routes walk calls to the Booster Robotics SDK to use the K1's built-in locomotion.
## Usage

Include this module to allow the robot to walk.

## Consumes

- `message::skill::Walk` A Task requesting to walk, containing a vector with the desired velocity target.

## Emits

- `message::behaviour::state::Stability` to indicate when the robot is walking (dynamically stable) and standing.

## Dependencies

- Director
- Eigen
