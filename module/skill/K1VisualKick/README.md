# K1VisualKick

## Description

Sends a command to the Booster Robotics SDK to initiate a visual kick.

## Usage

Emit a Kick message when wanting to kick.

## Consumes

- `message::skill::Kick` a request to kick. The Booster visual kick is autonomous

## Emits

- `message::booster::BoosterVisualKick` which the Booster `HardwareIO` forwards to the SDK via `booster_client.VisualKick(start, version)`.

## Dependencies

- Director
