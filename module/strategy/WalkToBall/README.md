# WalkToBall

## Description

Four types of walking to the ball exist in this module.

`WalkToBall` is a very simple Provider directly walking to the ball without considering the goal position.

`WalkToKickBall` walks the ball with the robot positioned such that it will move the ball towards the goals. It will move behind the ball first if the robot is positioned between the ball and goals.

`TackleBall` will walk to the ball perpendicularly to the opponent robot closest to the ball, tackling it off them.

`PositionBehindBall` will walk behind the ball, facing the goals, and stop behind the ball without touching it. This is for penalty positioning where the ball should not be touched yet.

## Usage

Add this module and emit the Task for the desired type of walk to ball.

## Consumes

- `message::strategy::WalkToBall` Task requesting to walk to the ball directly
- `message::strategy::WalkToKickBall` Task requesting to walk to the ball with the goals aligned
- `message::strategy::TackleBall` Task requesting to tackle the ball off an opponent
- `message::strategy::PositionBehindBall` Task requesting to position behind the ball without touching the ball, in line with the goals
- `message::localisation::Ball` information on where the ball is
- `message::support::FieldDescription` to know where the goals are
- `message::input::Sensors` for transforming into robot space
- `message::localisation::Field` for calculations in field space
- `message::localisation::Robots` used by TackleBall to find the opponent robot to tackle off

## Emits

- `message::planning::WalkTo` Task requesting to walk to the ball directly
- `message::strategy::WalkToFieldPosition` used by all except `WalkToBall` to position at the ball appropriately

## Dependencies

- `utility::math::euler::pos_rpy_to_transform` for creating the transform for `WalkToFieldPosition`
