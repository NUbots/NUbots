# TeamBehaviourTester

## Description

Drives the FieldPlayer team behaviour logic for the `test/teambehaviour` role without letting it control the
robot.

Every tick it synthesises a "we are playing a normal game" `GameState` (PLAYING phase, not stopped,
unpenalised) and requests the `FieldPlayer` purpose. This makes `FieldPlayer` run its real in-game decision
logic - based on the localised ball, teammates and field - and emit the `Purpose` (the role it would take:
attack, defend, support, etc.) and, when supporting, a `SupportPosition`. Both are forwarded to NUsight for
visualisation.

The robot itself is still driven around manually by `KeyboardWalk`. `FieldPlayer`'s own locomotion tasks
(Attack, Defend, Support, ...) are deliberately left without providers in the role, so they are inert and the
robot never moves on its own - this module only reports what role the robot *would* take if it were in a real
game.

## Usage

Include this module in a role alongside `purpose::FieldPlayer`, `extension::Director`, localisation and
`purpose::KeyboardWalk`. See `roles/test/teambehaviour.role`.

## Consumes

- `message::input::GameState` and `message::input::GameState::Phase` to trigger the FieldPlayer decision logic.

## Emits

- `message::input::GameState` - a synthetic PLAYING game state.
- `message::input::GameState::Phase` - the PLAYING phase guard read by FieldPlayer.
- `message::purpose::FieldPlayer` as a Task to activate the field player purpose.

## Dependencies

- `extension::Director`
- `module::purpose::FieldPlayer`
