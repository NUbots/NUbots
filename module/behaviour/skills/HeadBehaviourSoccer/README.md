# HeadBehaviourSoccer

## Description

Controls the state of the head

## Usage

Include this module to allow the robot to focus on a ball or search for a ball if it is not visible

## Consumes

- `message::localisation::FilteredBall` uses filtered ball position measurement to calculate angles to rotate head
- `using message::motion::ExecuteGetup` used to check if robot is getting up
- `using message::motion::KillGetup` used to check if the robot has finished getting up

## Emits

- `message::motion::HeadCommand` instructs the head to move to desired angle

## Dependencies

- `Eigen`
