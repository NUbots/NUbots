# Pong

## Description

Demonstration module that initiates and responds to Ping messages. Starts the ping-pong message chain on startup.

## Usage

Configuration from `Pong.yaml`

## Consumes

- `message::onboarding::Ping`

## Emits

- `message::onboarding::Pong` (on startup and in response to Ping)

## Dependencies

- `message::onboarding::Ping`
- `message::onboarding::Pong`
