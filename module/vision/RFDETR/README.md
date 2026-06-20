# RFDETR

## Description

This module integrates an **RF-DETR** (Roboflow Detection Transformer) model to
identify and classify objects within images. It is the successor to the `Yolo`
module (YOLOv8-nano) and detects the same classes, in the same order:

- Balls
- Goals
- Robots
- Field Line Intersections (L, T, X)

Confidence thresholds for each class can be specified in the config.

Inference runs on the CPU or GPU using OpenVINO
(https://github.com/openvinotoolkit/openvino), loading the exported ONNX model
directly.

### How it differs from `Yolo`

RF-DETR is a DETR-style detector, so the network contract differs from YOLOv8 and
the decoder/preprocessing are not interchangeable:

- **Two output tensors** — `pred_boxes [1, Q, 4]` (normalized `cxcywh`) and
  `pred_logits [1, Q, num_classes]`. Detections are obtained by `sigmoid` +
  thresholding the logits; there is **no NMS**.
- **Preprocessing** — square resize → scale to `[0, 1]` → ImageNet `mean/std`
  normalisation → NCHW, RGB. This must match the training/export transform
  exactly (see `../training/verify_onnx.py`).
- **Class-channel offset** — the exported model carries an extra (unused)
  background channel, so `num_classes` is one larger than the number of objects.
  The module auto-detects this (`class_offset: -1`).

Everything downstream of detection (ray projection onto the field plane, message
emission, green-horizon filtering) is identical to `Yolo`, so consumers
(`BallLocalisation`, `RobotLocalisation`, `FieldLocalisationNLopt`) are unchanged.

## Training

The ONNX model is produced by the offline RF-DETR training pipeline in
`../training/` (outside the NUbots repo). It fine-tunes a pretrained RF-DETR on
the [`NUbots/YOLOSoccer`](https://huggingface.co/datasets/NUbots/YOLOSoccer)
dataset and installs `data/rfdetr.onnx`. See `../training/README.md`.

## Usage

Include this module to detect balls, goals, robots and field line intersections
in images. Add `vision::RFDETR` to a role in place of `vision::Yolo`.

If the GreenHorizon is included in the program, balls, field line intersections
and robots outside of the GreenHorizon will be discarded.

To run with the GPU device in docker you need the flags `./b run {binary} --gpus all`.

## Consumes

- `message::input::Image` the image to run RF-DETR on.
- `message::vision::GreenHorizon` (optional) used to discard detections outside
  the field.

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
