# RoboCup

## Description

Starts the Director graph for the RoboCup scenario. If the robot is penalised, it will make it stand still. If force play is set, it will bypass the higher level position Providers and skip to the Play Providers. Similarly for force penalty shootout, the PenaltyShootout Providers will be requested. Otherwise, the position Provider will run based on the set position.

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
- `message::purpose::PlayStriker` a task to Request the robot acts as a striker and bypass GameController to assume the play state
- `message::purpose::PenaltyShootoutStriker` a task to Request the robot acts as a striker and bypasses GameController to assume the penalty shootout state

- `message::purpose::Goalie` a task to Request the robot acts as a goalie
- `message::purpose::PlayGoalie` a task to Request the robot acts as a goalie and bypass GameController to assume the play state
- `message::purpose::PenaltyShootoutGoalie` a task to Request the robot acts as a goalie and bypass GameController to assume the penalty shootout state

- `message::purpose::Defender` a task to Request the robot acts as a defender
- `message::purpose::PlayDefender` a task to Request the robot acts as a defender and bypass GameController to assume the play state

## Dependencies
