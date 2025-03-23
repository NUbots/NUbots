import os
import torch
import argparse
from train import EfficientSegmentationModel


def convert_to_onnx(model_path, output_path, img_size=512):
    """
    Convert a PyTorch model to ONNX format

    Args:
        model_path: Path to the saved PyTorch model
        output_path: Path where the ONNX model will be saved
        img_size: Input image size for the model
    """
    # Create output directory if it doesn't exist
    os.makedirs(os.path.dirname(output_path), exist_ok=True)

    # Set device
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"Using device: {device}")

    # Initialize model
    model = EfficientSegmentationModel(in_channels=3, num_classes=3)

    # Load model weights
    model.load_state_dict(torch.load(model_path, map_location=device))
    model.to(device)
    model.eval()

    # Create dummy input for ONNX export
    dummy_input = torch.randn(1, 3, img_size, img_size).to(device)

    # Export to ONNX
    torch.onnx.export(
        model,
        dummy_input,
        output_path,
        export_params=True,
        opset_version=11,
        input_names=["input"],
        output_names=["output"],
        dynamic_axes={"input": {0: "batch_size"}, "output": {0: "batch_size"}},
    )

    print(f"Model successfully converted to ONNX format and saved at: {output_path}")


def main():
    parser = argparse.ArgumentParser(description="Convert PyTorch model to ONNX")
    parser.add_argument(
        "--model_path",
        type=str,
        default="results/best_segmentation_model.pth",
        help="Path to the saved PyTorch model",
    )
    parser.add_argument(
        "--output_path",
        type=str,
        default="results/segmentation_model.onnx",
        help="Path where the ONNX model will be saved",
    )
    parser.add_argument(
        "--img_size", type=int, default=512, help="Input image size for the model"
    )

    args = parser.parse_args()

    convert_to_onnx(args.model_path, args.output_path, args.img_size)


if __name__ == "__main__":
    main()
