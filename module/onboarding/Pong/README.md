# Pong

## Description

Handles the receiving of a `Ping` message with the previous count.

## Usage

Add this module to the role and emit a `Pong` message to initiate the interaction. The module get the ping message's `count` each time it receives a `Ping` message and emits the updated `Pong`.

## Consumes

- `message::onboarding::Ping`: A message received from the Ping module.

## Emits

- `message::onboarding::Pong`: A message containing a `count`, sent to the Ping module.

## Dependencies

- Message types for communication with the Ping module. (?)
