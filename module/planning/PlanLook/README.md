# PlanLook

## Description

Plans how to look for features.

LookAtBall looks for the ball and fixates on it if it has been seen.
LookAtGoals looks for goals and fixates on them if they have been seen.
LookAround performs a search pattern to look around the environment.

## Usage

Add this module to your role to make the robot look for requested features.

## Consumes

- `message::planning::LookAtBall` a Task requesting to look at the ball and search if it can't see it
- `message::planning::LookAtGoals` a Task requesting to look at the goals and search if it can't see any
- `message::localisation::FilteredBall` with the position of the ball for fixation and knowing if we've seen the ball
- `message::vision::Goals` with the position of the goals for fixation and knowing if we've seen goals
- `message::input::Sensors` for the world matrix to convert the goals measurement into the right space
- `message::planning::LookAround` a Task requesting to run the search pattern to look around

## Emits

- `message::skill::Look` a Task requesting to look in a direction
- `message::planning::LookAround` a Task requesting to run the search pattern to look around the environment

## Dependencies

- The coordinates utility
