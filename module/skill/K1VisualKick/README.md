# K1VisualKick

## Description

Sends a command to the Booster Robotics SDK to initiate a visual kick.

## Usage

Emit a Kick message when wanting to kick.

## Consumes

- `message::skill::Kick` the kick information to use when kicking.

## Emits

- `message::booster::BoosterVisualKick` through the Script extension, which populates the requested Script/s into LimbsSequences.

## Dependencies

- Director
