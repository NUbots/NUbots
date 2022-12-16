# FallRecovery

## Description

A strategy module that handles relaxing when falling and getting up when we hit the ground and have settled.
Ensures that the GetUp planner has higher priority than the FallingRelax planner so it doesn't fail during GetUp.

It is recommended that this task be given higher priority than any other running provider in the system so nothing can interrupt it.

## Usage

Emit a `message::strategy::FallRecovery` Task in order to activate this module's provider

## Consumes

- `message::strategy::FallRecovery` to activate the provider

## Emits

- `message::planning::PlanGetUp` to activate the Get Up planner
- `message::planning::PlanFallingRelax` to activate the FallingRelax planner
