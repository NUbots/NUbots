# AlignBallToGoal

## Description

Strafes in a circle around the ball if the ball exists.

## Usage

Add this module to align the ball to the goal.

## Consumes

- `message::strategy::AlignBallToGoal` Task requesting to rotate around the ball.
- `message::Localisation::Ball` Information on where the ball is.
- `message::Localisation::Field` Information regarding the position of the field.
- `message::support::FieldDescription` Information on the dimensions of the field, used to find the goal position to align to.

## Emits

- `message::planning::TurnAroundBall` Task requesting to rotate around the ball.

## Dependencies
