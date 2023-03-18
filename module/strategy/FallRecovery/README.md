# FallRecovery

## Description

A strategy module that handles relaxing when falling and getting up when we hit the ground and have settled.
Ensures that getting up has higher priority than falling planner so it doesn't try to relax while getting up.

It is recommended that this task be given higher priority than any other running provider in the system so nothing can interrupt it.

## Usage

Emit a `message::strategy::FallRecovery` Task in order to activate this module's provider

## Consumes

- `message::strategy::FallRecovery` to activate the provider

## Emits

- `message::planning::GetUpWhenFalling` to activate the get up planner
- `message::planning::RelaxWhenFalling` to activate a relax planner
