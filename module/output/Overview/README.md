# Overview

## Description

Builds a periodic high-level status packet for NUsight.

The module combines localisation, game state, walk/kick intent, battery/voltage, and recent-vision timing into a
single `Overview` message that can be forwarded to NUsight.

## Usage

Install this module together with the producers of the input messages below.

The module:

- Emits an overview every 2 seconds
- Tracks most recent camera/ball/goal observation times from trigger events
- Emits the message to NUsight through `NetworkForwarder`.

## Consumes

- `message::support::GlobalConfig` (optional): player id
- `NUClear::message::CommandLineArguments` (optional): role/binary name
- `message::input::Sensors` (optional): battery, voltage, robot transform
- `message::localisation::Field` (optional): robot field transform and covariance
- `message::localisation::Ball` (optional): ball field position and covariance
- `message::skill::Kick` (optional): kick target
- `message::input::GameState` (optional): game mode, phase, penalty reason
- `message::skill::Walk` (optional): walk velocity command
- `message::input::Image` (trigger): updates last camera image time
- `message::vision::Balls` (trigger): updates last seen ball time when non-empty
- `message::vision::Goals` (trigger): updates last seen goal time when non-empty

## Emits

- `message::support::nusight::Overview`
