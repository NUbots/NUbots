# Yolo

## Description

This module integrates a YOLO (You Only Look Once) model to identify and classify objects within images. The default model is the K1 RoboCup detector (`booster.onnx`). Its classes are:

- `Ball`
- `Goalpost`
- `K1` (treated as a robot detection)
- `LCross`, `TCross`, `XCross` (field line intersections)
- `PenaltyPoint`

Confidence thresholds for each class can be specified in the config.

Penalty point detections have no dedicated message type yet, so they are only emitted as a `message::vision::BoundingBox` for visualisation/debugging in NUsight.

Inference is run using ONNX Runtime (https://onnxruntime.ai/), either using the default CPU execution provider, or by using the TensorRT execution provider for NVIDIA GPUs.

Note that inference using non-NVIDIA GPUs is currently unsupported.

## Usage

Include this module to detect balls, goals, robots, field line intersections and penalty points in images.

NOTE: If you are running a model for the first time on a robot using the TensorRT execution provider, it may take a few minutes for the model to load, as the EP parses the ONNX file into a format that TensorRT can run. This does not happen on subsequent runs.

## Consumes

- `message::input::Image` the image to run the YOLO on.

## Emits

- `message::vision::Balls` ball detections
- `message::vision::Goals` goal detections
- `message::vision::Robots` robot detections (from the "K1" class)
- `message::vision::FieldIntersections` field line intersections
- `message::vision::BoundingBoxes` bounding boxes for every detected class, including penalty points

## Dependencies

- [ONNX Runtime](https://onnxruntime.ai/)
- [Eigen Linear Algebra Library](https://eigen.tuxfamily.org/index.php)
- [OpenCV](https://opencv.org/)
