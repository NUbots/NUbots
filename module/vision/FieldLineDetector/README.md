# FieldLineDetector

## Description

The FieldLineDetector module performs post-processing on the output of the VisualMesh to determine where field lines are in the image, if any. If there are no field lines, it will emit an empty `FieldLines` message.

Each FieldLine detection is checked to see if it is below the green horizon. Zero or many FieldLines could be detected and emitted in a FieldLines message.

## Usage

Add this module to get information about FieldLines, given the required messages are present.

## Consumes

- `message::vision::GreenHorizon` which contains points that represent the green horizon, and the Visual Mesh output. The Visual Mesh output is used to extract the field line detections and the green horizon is used to remove field line detections above the horizon.

## Emits

- `message::vision::FieldLines` which contains information such as transform to the camera from world, and the unit vectors of field line detections in world space.

## Dependencies

- [Eigen Linear Algebra Library](https://eigen.tuxfamily.org/index.php)
