BallDetector
============

## Description

The BallDetector module performs post-processing on the output of the VisualMesh to determine where balls are in the image, if any.

## Usage

## Consumes

- `message::vision::GreenHorizon` which contains points that represent the green horizon, and the Visual Mesh output. The Visual Mesh output is used to create the ball clusters and the green horizon is used to remove balls above the horizon.
- `message::support::FieldDescription` which contains parameters that define the structure and size of the field. This is used to determine if a detection of a ball is likely to be a ball.

## Emits

- `message::vision::Balls` which contains an array of `message::vision::Ball` messages, which contains information such as position and covariance of the ball. Can be empty.

## Dependencies

- [Eigen Linear Algebra Library](https://eigen.tuxfamily.org/index.php)