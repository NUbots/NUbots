# BallDetector

## Description

The BallDetector module performs post-processing on the output of the VisualMesh to determine where balls are in the image, if any. If there are no balls, it will emit an empty `Balls` message.

The BallDetector first cluster points, creates ball candidates and then adds candidates to balls if they pass a set of checks.

Clusters are created by clustering neighbouring Visual Mesh points that have a high enough confidence that they are a ball point and are not surrounded by ball points. That is, clusters are made up of ball edge points. Ball candidates are made of a vector of cluster indices and metadata like radius and central axis. For each cluster a ball candidate is created which corresponds to it. Close together ball candidates are checked for if merging improves circular fit, if it does a new ball candidate is created holding the cluster indices of all candidates that were used in merging. Ball candidates that were used in merging are marked.

Each candidate is checked to see if

1. It wasn't marked as used in merging
2. It is below or intersecting the green horizon
3. It has a high enough degree of fit to a circle
4. It's angular and projection based distances are close enough
5. It is not too close to the robot
6. It is closer or equal to the length of the field

If it meet all of these criterion, the candidate is rejected

If a candidate passes the checks, it is added to a Balls message.

Zero or many balls could be detected and emitted in a Balls message.

## Usage

Add this module to get information about balls, given the required messages are present.

## Consumes

- `message::vision::GreenHorizon` which contains points that represent the green horizon, and the Visual Mesh output. The Visual Mesh output is used to create the ball clusters and the green horizon is used to remove balls above the horizon.
- `message::support::FieldDescription` which contains parameters that define the structure of the field and size of the ball. This is used in the checks mentioned above, to determine if the ball candidate is likely to be a ball.

## Emits

- `message::vision::Balls` which contains an array of `message::vision::Ball` messages, which contains information such as position and covariance of the ball. Can be empty.

## Dependencies

- [Eigen Linear Algebra Library](https://eigen.tuxfamily.org/index.php)
