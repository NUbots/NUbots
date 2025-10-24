# Ping

## Description

This module is one half of a pair of modules that receive a message containing a counter, increment the counter, and send a response message with the new counter.

## Usage

Include onboarding::Ping as well as onboarding::Pong in your role.

## Consumes

- `message::onboarding::Pong` Pong message containing a counter

## Emits

- `message::onboarding::Ping` Response to Pong module containing the received counter incremented by 1

## Dependencies

Depends on complementary onboarding::Pong module.
