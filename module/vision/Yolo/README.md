# Yolo

## Description

This module integrates a YOLO (You Only Look Once) model to identify and classify objects within images. The classes for the model are:

- Balls
- Goals
- Robots
- Field Line Intersections (L,T,X)

Confidence thresholds for each class can be specified in the config.

Inference can be ran on either the CPU or GPU using OpenVino (https://github.com/openvinotoolkit/openvino).

## Usage

Include this module to detect balls, goals, robots and field line intersections in images.

If the GreenHorizon is included in the program, balls, field line intersections and robots outside of the GreenHorizon will be discarded.

To run with GPU device in docker you need to include the following flags `./b run {binary} --gpus all`

## Consumes

- `message::input::Image` the image to run the YOLO on.

## Emits

- `message::vision::Balls` ball detections
- `message::vision::Goals` goal detections
- `message::vision::Robots` robot detections
- `message::vision::FieldIntersections` field line intersections

## Dependencies

- [OpenVino](https://github.com/openvinotoolkit/openvino)
- [Eigen Linear Algebra Library](https://eigen.tuxfamily.org/index.php)
- [OpenCV](https://opencv.org/)
