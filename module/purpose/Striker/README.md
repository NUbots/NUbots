# Striker

## Description

Play soccer in the striker position, which is an offensive position. Either bypass GameController and force play, or uses GameController information to decide what to do.

In the ready state, makes a Ready subtask.

In the playing state (for both normal game mode and penalty shootout), finds the ball, fixates on the ball, walks to the ball, and kicks the ball. It will not approach the ball if it is not in the front half, and instead will stand nearby.

In other states, stands still.

## Usage

Add this module to play as a striker in soccer!

## Consumes

- `message::input::GameState` to get information about the state of the game
- `message::input::GameState::Data::Phase` to get specific information about the current game phase (initial, ready, set, playing, etc).
- `message::strategy::Striker` a Task requesting to play as a Striker
- `message::strategy::NormalStriker` a Test requesting to play as a Striker in the normal game state
- `message::strategy::PenaltyShootoutStriker` a Task requesting to play as a Striker in the penalty shootout game state

## Emits

- `message::strategy::NormalStriker` a Task requesting to play as a Striker in the normal game state
- `message::strategy::PenaltyShootoutStriker` a Task requesting to play as a Striker in the penalty shootout game state
- `message::strategy::Ready` a Task requesting to walk to the ready position
- `message::strategy::StandStill` a Task requesting to stand still and not move
- `message::strategy::FindBall` a Task requesting to look and move around to find the ball
- `message::strategy::LookAtBall` a Task requesting to look at a known ball
- `message::strategy::WalkToBall` a Task requesting to walk to a known ball
- `message::strategy::AlignBallToGoal` a Task requesting to align the robot to face the goal with the ball infront of it
- `message::planning::KickToGoal` a Task requesting to kick the ball towards the goal
- `message::strategy::WalkInsideBoundedBox` a Task requesting that the robot stays within a defined bounded box

## Dependencies
