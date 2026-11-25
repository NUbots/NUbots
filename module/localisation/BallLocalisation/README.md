# BallLocalisation

## Description

This module estimates the ball's position and velocity in world space with a constant-velocity Unscented Kalman Filter,
run in image time. Each detection has a range-scaled noise and is gated against the prediction by Mahalanobis distance.
Detections outside the gate are taken as a kick when the ball could have reached them and the previous image confirms
them, and a run of same-direction innovations inside the gate opens up the velocity covariance. Tracking starts on, and
reacquires after a loss to, a detection seen in two consecutive images. The published estimate is predicted forward over
vision's latency, and teammate balls are used as a fallback when the ball has not been seen recently.

## Usage

Include this module to allow the robot to estimate the balls position and velocity.

## Consumes

- `message::vision::Balls` uses the ball position estimate from vision
- `message::input::Sensors` uses sensors to compute transform from camera {c} to torso space {t}
- `message::support::FieldDescription` uses field description to obtain height of ball off the ground
- `message::localisation::GroundTruthMode` stops the ball estimate while the simulator is providing ground truth

## Emits

- `message::localisation::Ball` contains filtered ball position measurement
