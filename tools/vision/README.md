# Vision Tools

Tooling for the robot field-segmentation model. Everything (training, evaluation
and ONNX export) lives in a single script: [`segmentation.py`](segmentation.py).

## Model

`FieldSegNet` is a compact depthwise-separable U-Net designed to run in real time
on the robot CPU via OpenVINO. It has four 2x downsampling stages with skip
connections and bilinear upsampling, and predicts three classes:

| index | class      | mask intensity |
| ----- | ---------- | -------------- |
| 0     | field      | ~255           |
| 1     | field_line | ~127           |
| 2     | background | ~0             |

The loss is a weighted cross-entropy + Dice combination. Class weights are
estimated automatically (median-frequency balancing) to handle the strong class
imbalance (field lines are a tiny fraction of pixels).

Preprocessing matches the C++ module (`module/vision/Segmentation`): RGB input
scaled to `[0, 1]` and normalised with the ImageNet mean/std. The exported ONNX
takes a `[1, 3, H, W]` tensor named `input` and returns `[1, num_classes, H, W]`
logits named `output`.

## Usage

### Train

```bash
./b vision segmentation train --data_dir datasets/torso21 --gpus all
```

Useful options:

- `--epochs` (default 40)
- `--batch_size` (default 16)
- `--base_channels` (default 24) — model width/capacity
- `--lr` (default 1e-3)
- `--num_workers` (default 4; use `0` if the Docker shared-memory volume is small)
- `--export` — also write the ONNX model after training

The best checkpoint (by validation mean IoU) is saved to
`recordings/torso21/field_seg.pth`.

### Evaluate

```bash
./b vision segmentation eval --data_dir datasets/torso21
```

Prints pixel accuracy, mean IoU and per-class IoU / precision / recall.

### Export to ONNX

```bash
./b vision segmentation export
```

Writes `module/vision/Segmentation/data/segmentation_model.onnx` by default
(override with `--onnx_path`).

## Dataset structure

```
datasets/torso21/
  train/images/*.png
  train/segmentations/*.png
  test/images/*.png
  test/segmentations/*.png
```
