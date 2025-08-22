# PenaltyShootout

## Description

This module handles penalty shootout scenarios in robot soccer. It manages the robot's behavior during penalty kicks, with different actions depending on whether the robot is taking the penalty shot or acting as a goalkeeper.

When it's the robot's kick off (taking the penalty), the robot will search for the ball, walk to it, and attempt to kick it into the goal. When it's the opponent's kick off (acting as goalkeeper), the robot remains stationary on the goal line and tracks the ball with its head, as per RoboCup rules that prevent goalkeepers from moving off the line during penalty kicks.

## Usage

Include this module in a role and execute during penalty shootout game phases. The module automatically activates when the game state indicates a penalty shootout scenario. Press the middle button after robot placement for penalty taker or goalie to reset localisation.

## Consumes

- `message::input::GameState` to determine if it's our kick off or the opponent's
- `message::input::GameState::Phase` to specifically check for the playing phase during penalty shootouts
- `message::localisation::Ball` for ball detection and tracking during penalty execution
- `message::input::ButtonMiddleDown` and `message::input::ButtonMiddleUp` for localisation reset
- `message::support::FieldDescription` to calculate penalty mark and goal positions for localisation

## Emits

- `message::behaviour::state::Stability` initial stability state on startup
- `message::behaviour::state::WalkState` initial walk state on startup
- `message::skill::Walk` for basic walking and standing
- `message::skill::Look` for looking forward when idle and tracking the ball
- `message::planning::LookAround` when the ball cannot be found within the timeout period
- `message::strategy::WalkToKickBall` when it's our kick off, to approach and kick the ball
- `message::strategy::LookAtBall` to track the ball with the head
- `message::strategy::FallRecovery` to handle falls
- `message::localisation::PenaltyReset` for resetting robot position when middle button is pressed
- `message::output::Buzzer` for audio feedback when middle button is released
