# Soccer

## Description

Starts the Director graph for the soccer scenario. If the robot is penalised, it will make it stand still. Otherwise, it will emit the Task corresponding to the soccer position specified in configuration.

Possible positions are Striker, Goalie and Defender.

## Usage

Include this in your role to start the Director tree to play soccer.

## Consumes

- `message::purpose::FindPurpose` to find the right purpose to emit
- `message::input::GameEvents::Penalisation` to find out if the robot has been penalised and should stop moving
- `message::input::GameEvents::Unpenalisation` to find out if the robot has been unpenalised and can play again
- `message::input::ButtonMiddleDown` to force play with a middle button press

## Emits

- `message::purpose::FindPurpose` a Task to request to find the robot's soccer playing purpose, to start the Director graph to play soccer
- `message::strategy::StandStill` to make the robot still while penalised
- `message::platform::ResetWebotsServos` to reset the servos in Webots when penalised
- `message::behaviour::state::Stability` to set the robot's initial stability state to standing
- `message::strategy::FallRecovery` a Task to request the robot to manage falling
- `message::purpose::Striker` a Task to request the robot acts as a striker
- `message::purpose::Goalie` a Task to request the robot acts as a goalie
- `message::purpose::Defender` a Task to request the robot acts as a defender

## Dependencies
