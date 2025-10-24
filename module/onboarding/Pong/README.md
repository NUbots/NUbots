# Pong

## Description

This module is one half of a pair of modules that receive a message containing a counter, increment the counter, and send a response message with the new counter.

## Usage

Include onboarding::Pong as well as onboarding::Ping in your role.

## Consumes

- `message::onboarding::Ping` Ping message containing a counter

## Emits

- `message::onboarding::Pong` Response, and initial message, to Ping module containing the received counter incremented by 1

## Dependencies

Depends on complementary onboarding::Ping module.
