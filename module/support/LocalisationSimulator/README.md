# LocalisationSimulator

## Description

Utility module for inspecting localisation field transforms.

It periodically logs the `Hfw` transform matrix from `message::localisation::Field`, which is useful for simulator
debugging and sanity-checking localisation output.

## Usage

Include this module in a role when you want a simple heartbeat printout of the current field transform.

Every 3 seconds, if a `message::localisation::Field` is available, logs `Field.Hfw`

## Consumes

- `extension::Configuration` from `LocalisationSimulator.yaml`
- `message::localisation::Field`
