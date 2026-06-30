# RFDETR

## Description

This module integrates an **RF-DETR** (Roboflow Detection Transformer) model to
identify and classify objects within images. It detects the following classes, in the same order:

- Balls
- Goals
- Robots
- Field Line Intersections (L, T, X)

Confidence thresholds for each class can be specified in the RFDETR.yaml.

Inference runs on the CPU or GPU using OpenVINO
(https://github.com/openvinotoolkit/openvino), loading the exported ONNX model
directly.

## Usage

Include this module to detect balls, goals, robots and field line intersections
in images. Add `vision::RFDETR` to a role.

If the GreenHorizon is included in the program, balls, field line intersections
and robots outside of the GreenHorizon will be discarded.

## Consumes

- `message::input::Image` the image to run RF-DETR on.
- `message::vision::GreenHorizon` used to discard detections outside the field.

## Emits

- `message::vision::Balls` ball detections
- `message::vision::Goals` goal detections
- `message::vision::Robots` robot detections
- `message::vision::FieldIntersections` field line intersections
- `message::vision::BoundingBoxes` debug bounding boxes for NUsight

## Dependencies

- [OpenVino](https://github.com/openvinotoolkit/openvino)
- [Eigen Linear Algebra Library](https://eigen.tuxfamily.org/index.php)
- [OpenCV](https://opencv.org/)
