# Tester

## Description

This module allows the user to select various combinations of strategies and planners with different priorities for
testing purposes. For example, you may want to only test walking to the ball and kicking, which can be achieved by only
enabling the WalkToBal and KickTo tasks in the config.

## Usage

Add this module and enable/disable strategies and planners in the config which you wish to test.

## Consumes

## Emits

If enabled in the config the module emits the following.

- `message::strategy::StandStill` a Task requesting to stand still and not move
- `message::strategy::FindBall` a Task requesting to look and move around to find the ball
- `message::strategy::LookAtBall` a Task requesting to look at a known ball
- `message::strategy::WalkToBall` a Task requesting to walk to a known ball
- `message::strategy::AlignBallToGoal` a Task requesting to align the robot to face the goal with the ball infront of it
- `message::planning::KickToGoal` a Task requesting to kick the ball towards the goal
- `message::planning::KickTo` a Task requesting to kick the ball if close
- `message::strategy::StandStill` a Task requesting the robot to stand still
- `message::strategy::LookAround` a Task requesting the robot to look around
- `message::strategy::WalkToFieldPosition` a Task requesting the robot to walk to a specified position

## Dependencies

- Eigen
