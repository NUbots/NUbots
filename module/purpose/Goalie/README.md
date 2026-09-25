# Goalie

## Description

Play soccer in the goalie position.

In the ready state, walks to the goals.

In the playing state, the goalie never goes for the ball, so it never leaves the penalty area, even when it is the closest robot or has no teammates. While the ball is in the opponents' half it waits in the middle of the goal. While the ball is in our half it strafes along the goal line with the ball and emits `Save`, so `planning::PlanSave` positions it inside the penalty area and blocks shots with the block policy. Without PlanSave in the role (or with `save_priority: 0`), the strafe alone positions it.

If the ball is not visible it will look around without moving.

## Usage

Add this module to the role and emit a Goalie Task.

## Consumes

- `message::strategy::Goalie` a Task requesting to play as a Goalie
- `message::input::GameState` to get information about the state of the game, including penalties
- `message::input::GameState::Phase` to get specific information about the current game phase (initial, ready, set, playing, etc).
- `message::localisation::Ball` for determining if the ball is in our half, and act appropriately.
- `message::localisation::Field` to calculate in field space.
- `message::support::GlobalConfig` to get our own player ID.
- `message::support::FieldDescription` to calculate where the goals are for positioning.

## Emits

- `message::strategy::StandStill` a Task requesting to stand still and not move, outside of `READY` or `PLAYING`
- `message::planning:::LookAround` a Task requesting to look around for the ball
- `message::strategy::LookAtBall` a Task requesting to look at a known ball
- `message::strategy::WalkToFieldPosition` Task requesting to walk to position on field, for positioning at the goals
- `message::planning::Save` a Task requesting `planning::PlanSave` to guard the goal, while the ball is in our half
- `message::purpose::Purpose` information on the position the robot is playing (goalie), its ID and active state.

## Dependencies

- Director
