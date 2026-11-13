# BoosterWalk

## Description

Routes walk calls to the Booster Robotics SDK to use the K1's built-in locomotion. This is the legacy firmware walk;
`skill::Walk` runs the mjlab AMP walk policy instead. Only one walk skill should be in a role.

## Usage

Include this module to allow the robot to walk.

## Consumes

- `message::skill::Walk` A Task requesting to walk, containing a vector with the desired velocity target.

## Emits

- `message::behaviour::state::Stability` to indicate when the robot is walking (dynamically stable) and standing.

## Dependencies

- Director
- Eigen
