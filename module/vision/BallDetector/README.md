# BallDetector

## Description

The BallDetector module performs post-processing on the output of the VisualMesh to determine where balls are in the image, if any. If there are no balls, it will emit an empty `Balls` message.

The BallDetector first tries to create balls, and then removes candidate balls if they fail to pass a set of checks.

Balls are created by clustering neighbouring Visual Mesh points that have a high enough confidence that they are a ball point.

Each cluster is checked to see if

1. It is below the green horizon
2. It has a high enough degree of fit
3. It is not too close to the robot
4. It's angular and projection based distances are close enough
5. It is closer or equal to the length of the field

If it doesn't meet any of these criterion, the cluster is not considered to be a ball.

If a cluster passes the checks, it is added to a Balls message.

Zero or many balls could be detected and emitted in a Balls message.

## Usage

Add this module to get information about balls, given the required messages are present.

## Consumes

- `message::vision::GreenHorizon` which contains points that represent the green horizon, and the Visual Mesh output. The Visual Mesh output is used to create the ball clusters and the green horizon is used to remove balls above the horizon.
- `message::support::FieldDescription` which contains parameters that define the structure and size of the field. This is used in the checks mentioned above, to determine if the ball candidate is likely to be a ball.

## Emits

- `message::vision::Balls` which contains an array of `message::vision::Ball` messages, which contains information such as position and covariance of the ball. Can be empty.

## Dependencies

- [Eigen Linear Algebra Library](https://eigen.tuxfamily.org/index.php)
