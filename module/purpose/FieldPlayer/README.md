# FieldPlayer

A general soccer field player - not the goalie.

## Description

In the `READY` mode, the player will dynamically position based on the number of team mates on each side. One robot (closest to the field center on the left side) will position either inside or just outside of the center circle for kick off.

The FieldPlayer will find the ball if it can't see it, and it will look at the ball if it can.

The closest robot to the ball will become an Attack player.

The closest robot to our own goal that isn't an Attack player will Defend. Any other player will Support.

If it's the other team's kick off and the kick off time isn't up, the robot will wait.

In penalty positioning, the player will ReadyAttack. This will position the robot either offensively or defensively, ready for when play resumes.

## Usage

Use this module by including it in the role and emitting a `message::purpose::FieldPlayer` message to run the robot as a dynamic field player.

## Consumes

- `message::purpose::FieldPlayer` a task telling the robot to be a field player.
- `message::localisation::Ball` for determining who is closest to the ball, to become the Attack player.
- `message::localisation::Robots` to determine where team mates are and who has possession of the ball.
- `message::input::Sensors` to include ourselves in the list of robots.
- `message::localisation::Field` to calculate in field space.
- `message::input::GameState` for penalty and kick off information.
- `message::input::GameState::Phase` to know if it is the `READY` phase, or `PLAYING`, or another phase of the game.
- `message::support::GlobalConfig` to get our own player ID.
- `message::support::FieldDescription` to calculate ready positioning.

## Emits

- `message::strategy::FindBall` tells the robot to find the ball if it hasn't seen it in a while.
- `message::strategy::LookAtBall` tells the robot to look at the ball if it does know where it is.
- `message::strategy::Attack` tells the robot to act as the attacking player.
- `message::strategy::ReadyAttack` tells the robot to get ready to attack, but something is preventing it from actually attacking (penalty positioning, kick off).
- `message::strategy::Defend` tells the robot to hang back near the goals to defend.
- `message::strategy::Support` tells the robot it's not the attacking or defending player, it should instead act as support.
- `message::strategy::WalkToFieldPosition` task to walk to the calculated ready position.

## Dependencies

- `utility/strategy/positioning.hpp`
- `utility/strategy/soccer_strategy.hpp`
