# PlayGoalie

## Description

Plays goalie on a real robot without GameController (roles/goalie.role). It is the goalie's side of `purpose::Soccer`: after `start_delay` it emits `Goalie` with `FallRecovery` above it. It stands in for GameController with a GameState that is always in normal play, with this robot as the goalie.

`purpose::Goalie` never goes for the ball, so the goalie stays in the penalty area. `planning::PlanSave` positions it and switches to the block policy when a shot comes.

The buttons work as in `purpose::Soccer`. The left button pauses the goalie, which stands where it is. The middle button resets field localisation and starts it again after `start_delay`. So after the middle button, put the robot where `FieldLocalisationNLopt`'s `starting_side` expects it, as at the start of a game.

## Usage

Add this module to the role, without `purpose::Soccer` or `input::GameController`, which would fight it over the GameState.

## Consumes

- `message::input::ButtonLeftDown`, `ButtonMiddleDown` (and their `Up`s): pause and resume

## Emits

- `message::input::GameState` and `GameState::Phase`: always PLAYING, NORMAL mode, and this robot the goalie
- `message::purpose::Goalie` Task: play goalie
- `message::strategy::FallRecovery` Task: get up after a fall, above `Goalie`
- `message::skill::Walk` and `message::skill::Look` Tasks: stand still and look forward while idle
- `message::behaviour::state::Stability` and `WalkState`: initial states for the modules that wait on them
- `message::localisation::ResetFieldLocalisation`: on resume
- `message::output::Buzzer`: while a button is held

## Dependencies

- Director
- `purpose::Goalie` and the modules it needs (roles/goalie.role)
