# Ping

## Description

Handles the receiving of a `Pong` message with an incremented count.

## Usage

Add this module to the role and emit a `Ping` message in response to a `Pong` message. The module increments the `count` each time it receives a `Pong` message and emits the updated `Ping`.

## Consumes

- `message::onboarding::Pong`: A message received from the Pong module.

## Emits

- `message::onboarding::Ping`: A message containing an incremented `count`, sent to the Pong module.

## Dependencies

- Message types for communication with the Pong module. (?)
