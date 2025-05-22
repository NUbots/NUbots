# YoloCoco

## Description

This module integrates a YOLO (You Only Look Once) model to identify and classify objects within images.

The classes for the model are from COCO (https://docs.ultralytics.com/datasets/detect/coco/#dataset-structure)

Confidence thresholds for each class can be specified in the config.

Inference can be ran on either the CPU or GPU using OpenVino (https://github.com/openvinotoolkit/openvino).

## Usage

Include this module to detect balls, goals, robots and field line intersections in images.

To run with GPU device in docker you need to include the following flags `./b run {binary} --gpus all`

## Consumes

- `message::input::Image` the image to run the YOLO on.

## Emits

- `message::vision::BoundingBoxes` bounding boxes of the detections

## Dependencies

- [OpenVino](https://github.com/openvinotoolkit/openvino)
- [Eigen Linear Algebra Library](https://eigen.tuxfamily.org/index.php)
- [OpenCV](https://opencv.org/)
