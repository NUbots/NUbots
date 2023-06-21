# StrategiseLook

## Description

LookAtBall fixates on a ball if a recent one exists.
LookAtGoals fixates on a goal if a recent goal exists.

## Usage

Add this module to your role to make the robot look at balls or goals when requested.

## Consumes

- `message::planning::LookAtBall` a Task requesting to look at the ball if there is a recent ball
- `message::planning::LookAtGoals` a Task requesting to look at a goal if there is a recent goal
- `message::localisation::Ball` with the position of the ball for fixation
- `message::vision::Goals` with the position of the goals for fixation
- `message::input::Sensors` for the world matrix to convert the goals measurement into the correct space

## Emits

- `message::skill::Look` a Task requesting to look in a direction (to the balls or goals)

## Dependencies

- The coordinates utility
