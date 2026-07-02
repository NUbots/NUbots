# HSLLocalisationDebug

## Description

Builds a single, size-bounded packet for debugging localisation during an actual RoboCup Humanoid Soccer
League match.

The GameController's debug communication rule (RoboCupSoccer rules, S 3.9.5) limits a robot to sending at most one UDP packet per second of debug information to a team device. `NetworkForwarder` normally forwards each message type as its own independent packet, so forwarding field intersections, robots, ball, and field localisation individually would exceed that limit. This module combines them into one `HSLLocalisationDebug` message so `NetworkForwarder` only needs to send/rate-limit a single message type.

## Usage

Install this module together with the producers of the input messages below, and forward
`message.support.nusight.HSLLocalisationDebug` (and only that type, at a rate of `1`) to NUsight via
`NetworkForwarder`.

The module:

- Builds the combined packet at most once per second
- Fills each field from the latest available data, leaving it default/empty if that data hasn't arrived yet
- Reduces each `message::localisation::Robot` down to a `RobotSummary` (`id`/`rRWw`/`teammate` only), dropping
  `covariance`/`Hcw`/`Purpose`/timestamp: `Robots` was the single biggest contributor to packet size, and
  those extra fields aren't needed to plot a robot on the field
- Emits the message locally; `NetworkForwarder` is responsible for actually sending it to NUsight

## Consumes

- `message::vision::FieldIntersections` (optional)
- `message::localisation::Robots` (optional)
- `message::localisation::Ball` (optional)
- `message::localisation::Field` (optional)

## Emits

- `message::support::nusight::HSLLocalisationDebug`
