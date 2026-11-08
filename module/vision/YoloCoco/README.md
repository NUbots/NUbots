# YoloCoco

## Description

This module integrates a YOLO (You Only Look Once) model to identify and classify objects within images.

The classes for the model are from COCO (https://docs.ultralytics.com/datasets/detect/coco/#dataset-structure)

Confidence thresholds for each class can be specified in the config.

Inference is run using ONNX Runtime (https://onnxruntime.ai/), either using the default CPU execution provider, or by using the TensorRT execution provider for NVIDIA GPUs.

Note that inference using non-NVIDIA GPUs is currently unsupported.

## Usage

Include this module to detect balls, goals, robots and field line intersections in images.

NOTE: If you are running a model for the first time on a robot using the TensorRT execution provider, it may take a few minutes for the model to load, as the EP parses the ONNX file into a format that TensorRT can run. This does not happen on subsequent runs.

## Consumes

- `message::input::Image` the image to run the YOLO on.

## Emits

- `message::vision::BoundingBoxes` bounding boxes of the detections

## Dependencies

- [ONNX Runtime](https://onnxruntime.ai/)
- [Eigen Linear Algebra Library](https://eigen.tuxfamily.org/index.php)
- [OpenCV](https://opencv.org/)
