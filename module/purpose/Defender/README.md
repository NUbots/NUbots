# Defender

## Description

Play soccer in the defender position, which is an defensive position. Either bypass GameController and force play, or uses GameController information to decide what to do.

In the ready state, makes a Ready subtask.

In the playing state (for both normal game mode and penalty shootout), finds the ball, fixates on the ball, walks to the ball if the ball is in the defenders bounded box defensive area, and kicks the ball.

If ball is outside bounded box area and ball is in opposition half, defender is clamped to the edge of bounding box closest to ball.

If ball is outside bounded box area and ball is in own half, defender is clamped to the edge of bounding box closest to ball and position itself 1 meter behind the ball.

## Usage

Add this module to play as a defender in soccer!

## Consumes

- `message::input::GameState` to get information about the state of the game
- `message::input::GameState::Data::Phase` to get specific information about the current game phase (initial, ready, set, playing, etc).
- `message::strategy::WalkInsideBounded` a Task requesting to play as a Defender

## Emits

- `message::strategy::WalkInsideBoundedBox` a Task requesting to play as a defender in the normal game state clamped to bounded box
- `message::strategy::Ready` a Task requesting to walk to the ready position
- `message::strategy::StandStill` a Task requesting to stand still and not move
- `message::strategy::FindBall` a Task requesting to look and move around to find the ball
- `message::strategy::LookAtBall` a Task requesting to look at a known ball
- `message::strategy::WalkToBall` a Task requesting to walk to a known ball
- `message::strategy::AlignBallToGoal` a Task requesting to align the robot to face the goal with the ball in front of it
- `message::planning::KickToGoal` a Task requesting to kick the ball towards the goal

## Dependencies
