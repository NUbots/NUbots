# MocapToReal

## Description

Take motion capture from `MotionCapture.proto`, interpolate the skeleton information from human proportion to robot angles, then write it out to the robot. This would be for imitation learning or for demos of movement from MotionCapture.

## Usage


## Consumes

- `message.input.MotionCapture`

## Emits

- `message.actuation.ServoTarget`

## Dependencies
