# Soccer

## Description

Coordinates the high-level soccer behaviour state machine.

This module manages when the robot should actively play, idle, or pause due to penalties, then starts the purpose-selection flow and persistent
recovery behaviours.

Key responsibilities:

- Startup initialisation (`Stability`, `WalkState`, idle stand/look)
- Starting purpose selection (`FindPurpose`) and always-on fall recovery
- Selecting `Goalie` or `FieldPlayer` based on `GameState.self.goalie`
- Handling self penalisation/unpenalisation transitions
- Handling button-based idle enable/disable flow with configurable delay

## Usage

Include this module in a soccer role.

Configuration is read from `Soccer.yaml`:

- `force_playing`: if true, forces `GameState::Phase::PLAYING`
- `disable_idle_delay`: delay (seconds) before exiting idle after middle button press
- `startup_delay`: delay (seconds) before starting soccer behaviours after startup

Control flow:

- Left button (`ButtonLeftDown`) enables idle mode (pauses soccer tasks)
- Middle button (`ButtonMiddleDown`) disables idle mode after `disable_idle_delay`
- On penalisation (`GameEvents::Penalisation` for self), purpose tasks are cancelled and localisation/servo state is reset
- On unpenalisation, purpose and fall-recovery tasks are restarted (unless idling)

## Consumes

- `extension::Configuration` from `Soccer.yaml`
- `message::input::GameState`
- `message::support::GlobalConfig`
- `message::input::GameEvents::Penalisation`
- `message::input::GameEvents::Unpenalisation`
- `message::input::ButtonLeftDown`
- `message::input::ButtonLeftUp`
- `message::input::ButtonMiddleDown`
- `message::input::ButtonMiddleUp`

## Emits

- `message::behaviour::state::Stability`
- `message::behaviour::state::WalkState`
- `message::skill::Walk`
- `message::skill::Look`
- `message::purpose::FindPurpose`
- `message::purpose::FieldPlayer`
- `message::purpose::Goalie`
- `message::strategy::FallRecovery`
- `message::purpose::Purpose`
- `message::localisation::ResetFieldLocalisation`
- `message::platform::ResetWebotsServos`
- `message::output::Buzzer`

## Dependencies

- Director
