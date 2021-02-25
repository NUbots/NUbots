# StaticWalk

## Description

This walk engine moves the robot's torso so that its center of mass is over its support foot. Then it tells the swing foot to move to a target in front so as to move at the speed given by the walk command. It then repeats these steps with the old swing foot as the new support foot.

## Usage

## Emits

TorsoTarget
FootTarget

## Dependencies

Inverse kinematics
Eigen library
Sensors
