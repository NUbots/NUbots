# RoboCup

## Description

Starts the Director graph for the RoboCup scenario. If the robot is penalised, it will make it stand still. Otherwise, it will emit the Task corresponding to the soccer position specified in configuration.

Possible positions are Striker, Goalie and Defender.

## Usage

Include this in your role to start the Director tree for the RoboCup scenario.

## Consumes

- `message::purpose::FindPurpose` to find the right purpose to emit
- `message::input::GameEvents::Penalisation` to find out if the robot has been penalised and should stop moving
- `message::input::GameEvents::Unpenalisation` to find out if the robot has been unpenalised and can play again
- `message::platform::ButtonMiddleDown` to force play with a middle button press

## Emits

- `message::purpose::FindPurpose` a Task to request to find the robot's soccer playing purpose, to start the Director graph for the RoboCup scenario
- `message::strategy::StandStill` to make the robot still while penalised
- `message::platform::ResetWebotsServos` to reset the servos in Webots when penalised
- `message::purpose::Striker` a task to Request the robot acts as a striker
- `message::purpose::Goalie` a task to Request the robot acts as a goalie
- `message::purpose::Defender` a task to Request the robot acts as a defender
-
## Dependencies
