# WalkToBall

## Description

Walks to the ball if a filtered ball exists, otherwise walks in a search pattern to find the ball.

## Usage

Add this module to walk to the ball.

## Consumes

- `message::strategy::WalkToBall` Task requesting to walk to the ball
- `message::localisation::FilteredBall` information on where the ball is

## Emits

- `message::planning::WalkTo` Task requesting to walk to a location, in this case the ball
- `message::planning::TurnOnSpot` Task requesting to turn on the spot, to search for the ball

## Dependencies
