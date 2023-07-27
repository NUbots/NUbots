# FindObject

## Description

Provider reactions to find objects. Currently includes a FindBall Provider to find an ball when its location is unknown.

## Usage

Include this module and request a FindBall Task when needing to find the ball.

## Consumes

- `message::strategy::FindBall` a Task requesting to find the ball

## Emits

- `message::planning::TurnOnSpot` to turn around if the ball is not in view
- `message::planning::LookAround` to move the head around to find the ball

## Dependencies
