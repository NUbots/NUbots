# AllRounder

## Description

Play soccer in the all rounder position, which is a multi-purpose position for when only one robot is active. Either bypass GameController and force play, or uses GameController information to decide what to do.

In the ready state, makes a Ready subtask.

In the playing state (for both normal game mode and penalty shootout), finds the ball, fixates on the ball, walks to the ball, and kicks the ball.

In other states, stands still.

## Usage

Add this module to play as a all rounder in soccer!

## Consumes

- `message::input::GameState` to get information about the state of the game
- `message::input::GameState::Data::Phase` to get specific information about the current game phase (initial, ready, set, playing, etc).
- `message::strategy::AllRounder` a Task requesting to play as a AllRounder
- `message::strategy::NormalAllRounder` a Test requesting to play as a AllRounder in the normal game state
- `message::strategy::PenaltyShootoutAllRounder` a Task requesting to play as a AllRounder in the penalty shootout game state

## Emits

- `message::strategy::NormalAllRounder` a Task requesting to play as a AllRounder in the normal game state
- `message::strategy::PenaltyShootoutAllRounder` a Task requesting to play as a AllRounder in the penalty shootout game state
- `message::strategy::Ready` a Task requesting to walk to the ready position
- `message::strategy::StandStill` a Task requesting to stand still and not move
- `message::strategy::FindBall` a Task requesting to look and move around to find the ball
- `message::strategy::LookAtBall` a Task requesting to look at a known ball
- `message::strategy::WalkToBall` a Task requesting to walk to a known ball
- `message::strategy::AlignBallToGoal` a Task requesting to align the robot to face the goal with the ball infront of it
- `message::planning::KickToGoal` a Task requesting to kick the ball towards the goal

## Dependencies
