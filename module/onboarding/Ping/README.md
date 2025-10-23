# Ping

## Description

Demonstration module that responds to Pong messages. Increments and logs a counter value with each ping.

## Usage

Configuration from `Ping.yaml`

## Consumes

- `message::onboarding::Pong`

## Emits

- `message::onboarding::Ping` with incremented counter

## Dependencies

- `message::onboarding::Ping`
- `message::onboarding::Pong`
