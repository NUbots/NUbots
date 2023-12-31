# RobotDetector

## Description

This module uses segmentation information from an image to determine where other robots are in the world.

Clusters of robot detections are each checked to see if they are below or over the field convex hull, if enough points make up the cluster, and if the cluster is fair enough away (as we might classify our own legs/arms and don't want to consider that). Position is determined very basically by taking the closest point to the camera.

## Usage

Use this module to get raw positions of robots.

## Consumes

- `message::vision::GreenHorizon` containing the convex hull and Visual Mesh information used to find robots.

## Emits

- `message::vision::Robots` containing the positions of robots, if any.

## Dependencies

- [Eigen Linear Algebra Library](https://eigen.tuxfamily.org/index.php)
