# Vision Tools

This directory contains tools for training and testing a segmentation model for robot vision.

## Model Architecture

The model uses an efficient segmentation architecture based on MobileNetV3-inspired blocks with:
- Depthwise separable convolutions
- Squeeze-and-Excitation blocks for channel attention
- Skip connections for better feature preservation

## Usage

### Training

To train the model:
```bash
./b vision train --gpus all
```

Optional training parameters:
- `--img_size`: Input image size (default: 512)
- `--batch_size`: Training batch size (default: 8)
- `--num_epochs`: Number of training epochs (default: 30)
- `--lr`: Learning rate (default: 0.001)

### Testing

To test the model on an image:
```bash
./b vision test --test_image_path <path_to_image>
```

### Converting to ONNX

To convert the trained model to ONNX format:
```bash
./b vision convert_to_onnx
```

## Dataset Structure

The model expects the following directory structure:
