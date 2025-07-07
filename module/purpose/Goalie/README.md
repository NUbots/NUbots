# Goalie

## Description

Play soccer in the goalie position.

In the ready state, walks to the goals.

In the playing state, the goalie will typically stay within the goals unless the ball enters the defending third. In this case, it will either kick it back out (if the closest player) or it will stay between the ball and goals if another player is closer. In penalty states, it will freeze when appropriate and position for attacking if the ball is in the defending third and it is the closest to the ball.

If there are no teammates, it will act as a normal FieldPlayer.

If the ball is not visible it will look around without moving.

## Usage

Add this module to the role and emit a Goalie Task.

## Consumes

- `message::strategy::Goalie` a Task requesting to play as a Goalie
- `message::input::GameState` to get information about the state of the game, including penalties
- `message::input::GameState::Phase` to get specific information about the current game phase (initial, ready, set, playing, etc).
- `message::localisation::Ball` for determining if the ball is in the defending third, and act appropriately.
- `message::localisation::Robots` to determine where team mates are.
- `message::input::Sensors` to include ourself in possession and distance calculations
- `message::localisation::Field` to calculate in field space.
- `message::support::GlobalConfig` to get our own player ID.
- `message::support::FieldDescription` to calculate where the goals are for positioning.

## Emits

- `message::strategy::StandStill` a Task requesting to stand still and not move, outside of `READY` or `PLAYING`
- `message::planning:::LookAround` a Task requesting to look around for the ball
- `message::strategy::LookAtBall` a Task requesting to look at a known ball
- `message::strategy::WalkToFieldPosition` Task requesting to walk to position on field, for positioning at the goals
- `message::strategy::Attack` tells the robot to act as the attacking player when the ball is close to the goals
- `message::strategy::ReadyAttack` tells the robot to get ready to attack, but something is preventing it from actually attacking (penalty positioning, kick off).
- `message::purpose::FieldPlayer` a task telling the robot to be a field player, when there are no teammates.
- `message::purpose::Purpose` information on the position the robot is playing (goalie), its ID and active state.

## Dependencies
