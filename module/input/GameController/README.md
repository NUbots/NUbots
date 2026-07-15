# Game Controller

## Description

Listens for RoboCup GameController UDP broadcast packets, converts them into NUClear messages, and maintains a canonical `message::input::GameState`.

The module also sends periodic reply packets back to the GameController.

## Usage

Install `module::input::GameController` in the powerplant.

This module is configured by `GameController.yaml` and consumes:

- `message::support::GlobalConfig` for `team_id` and `player_id`

At runtime it:

- Listens for UDP broadcast packets on `receive_port`
- Accepts only supported packet version `19`
- Optionally filters incoming packets by source IP using `udp_filter_address`
- Tracks the sender address and sends reply packets to that address on `send_port`
- Sends `ALIVE` reply packets every 2 seconds

## Emits

- `message::input::GameState`
- `message::input::GameState::Phase`
- `message::input::GameState::Mode`
- `message::input::GameState::TeamColour`
- `message::input::GameEvents::Score`
- `message::input::GameEvents::GoalScored`
- `message::input::GameEvents::Penalisation`
- `message::input::GameEvents::Unpenalisation`
- `message::input::GameEvents::HalfTime`
- `message::input::GameEvents::KickOffTeam`
- `message::input::GameEvents::GamePhase`
- `message::input::GameEvents::GameMode`

Also sends UDP GameController reply packets via `emit<Scope::UDP>(...)`.

## Dependencies

- NUClear UDP networking (for broadcast receive and unicast reply)
