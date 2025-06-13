# ReadyAttack

An attacking player who can't attack yet, and must get ready.

## Description

If it is kick off, we aren't the kick off team if we can't attack yet. Position outside the center circle and wait for play to properly start.

If it is a penalty state and we aren't the attacking team, position in line with our goal and the ball to defend at the distance back specified in the rules.

If we are the attacking team, walk around to position behind the ball, ready to play.

## Usage

Include this module and emit a ReadyAttack task if the robot cannot attack due to the game state.

## Consumes

- `message::purpose::ReadyAttack` a task telling the robot to ready for attacking.
- `message::input::GameState` to determine which team is taking the penalty.
- `message::support::FieldDescription` to determine positioning relative to the goal.
- `message::localisation::Field` for measurements in field space.
- `message::localisation::Ball` to position relative to the ball.

## Emits

- `message::strategy::WalkToFieldPosition` to position the robot on the field away from the ball, for defending situations.
- `message::strategy::PositionBehindBall` to position the robot behind the ball without kicking it.

## Dependencies

- `utility::math::euler::pos_rpy_to_transform` for field position matrix creation.
