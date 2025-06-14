# Attack

An attack-orientated robot position.

## Description

The attacker either dribbles the ball towards the goal to score or tackles the ball off an opponent if they have possession.

## Usage

Designed to be used by one robot as the main attacker.

## Consumes

- `message::purpose::Attack` a task telling the robot to be an attacking player.

## Emits

- `message::strategy::WalkToKickBall` when dribbling towards the goal.
- `message::strategy::TackleBall` when tackling the ball off an opponent.

## Dependencies

- None
