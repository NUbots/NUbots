# GreenHorizonDetector

## Description

Calculates a convex hull of the field from the Visual Mesh points.

1. Finds all the points that are on the field (ie high confidence of field or field line)
2. Clusters the points into groups of points next to each other
3. Finds the cluster that is closest to the robot, which is used in the remainder of the algorithm
4. Gets only the points that are on the boundary of the cluster, ie the boundary of the field
5. Calculates a convex hull (ie a minimal polygon) from that boundary

## Usage

Add this module to the role and when it gets a Visual Mesh message it will create a convex hull.
This convex hull is used in vision object detectors to exclude detections outside of the field.

## Consumes

- `message::vision::VisualMesh` a message containing classification of points on the ground plane of an image.

## Emits

- `message::vision::GreenHorizon` a message containing the Visual Mesh data and its associated convex hull of the field.

## Dependencies

- [Eigen Linear Algebra Library](https://eigen.tuxfamily.org/index.php)
