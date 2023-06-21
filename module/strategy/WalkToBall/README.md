# WalkToBall

## Description

Walks to the ball if a filtered ball exists.

## Usage

Add this module to walk to the ball.

## Consumes

- `message::strategy::WalkToBall` Task requesting to walk to the ball
- `message::localisation::Ball` information on where the ball is

## Emits

- `message::planning::WalkTo` Task requesting to walk to a location, in this case the ball

## Dependencies
