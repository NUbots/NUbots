# BallLocalisation

## Description

This module takes in a list of vision balls, uses the ball measurement closest to our current estimate and applies an
Unscented Kalman Filter to estimate the balls position and velocity in world space.

When the robot has not had a ball of its own accepted for `cooperative_ball_timeout` seconds, and
`use_cooperative_ball` is on, it falls back to the cooperative ball from `localisation::CooperativeLocalisation`, which is
fused from every teammate's detections, as long as someone has seen the ball within `cooperative_ball_max_age` seconds.
The fallback ball has a confidence of 0, and the filter is moved onto it, so the robot's next detection of the ball is
gated around where the team thinks the ball is. This replaces the old fallback to the average of teammates' reported
balls.

## Usage

Include this module to allow the robot to estimate the balls position and velocity.

## Consumes

- `message::vision::Balls` uses the ball position estimate from vision
- `message::input::Sensors` uses sensors to compute transform from camera {c} to torso space {t}
- `message::support::FieldDescription` uses field description to obtain height of ball off the ground
- `message::localisation::CooperativeBall` the cooperative ball, used as a backup when the ball has not been seen
  recently
- `message::localisation::GroundTruthMode` stops the ball estimate while the simulator is providing ground truth

## Emits

- `message::localisation::Ball` contains filtered ball position measurement
