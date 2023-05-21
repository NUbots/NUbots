# Goalie

## Description

Play soccer in the goalie position. Either bypasses GameController and force playing, or uses GameController information to decide what to do.

In the ready state, walks to the goals.

In the playing state (for both normal game mode and penalty shootout), finds the ball, fixates on the ball and dives if the ball is close.

## Usage

Add this module to the role and emit a Goalie Task.

## Consumes

- `message::input::GameState` to get information about the state of the game
- `message::input::GameState::Data::Phase` to get specific information about the current game phase (initial, ready, set, playing, etc).
- `message::strategy::Goalie` a Task requesting to play as a Goalie
- `message::strategy::NormalGoalie` a Test requesting to play as a Goalie in the normal game state
- `message::strategy::PenaltyShootoutGoalie` a Task requesting to play as a Goalie in the penalty shootout game state

## Emits

- `message::strategy::NormalGoalie` a Task requesting to play as a Goalie in the normal game state
- `message::strategy::PenaltyShootoutGoalie` a Task requesting to play as a Goalie in the penalty shootout game state
- `message::strategy::StandStill` a Task requesting to stand still and not move
- `message::planning:::LookAround` a Task requesting to look around for the ball
- `message::strategy::LookAtBall` a Task requesting to look at a known ball
- `message::strategy::DiveToBall` a Task requesting to dive to the ball if it is close enough
- `message::strategy::WalkToFieldPosition` Task requesting to walk to position on field

## Dependencies
