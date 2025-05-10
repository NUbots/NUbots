#!/usr/bin/env python3
#
# MIT License
#
# Copyright (c) 2025 NUbots
#
# This file is part of the NUbots codebase.
# See https://github.com/NUbots/NUbots for further info.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#

import argparse
import os

import torch
from train import EfficientSegmentationModel

# Assuming run_on_docker is available in your project structure
# If not, you might need to adjust the import or remove the decorator
from utility.dockerise import run_on_docker


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


@run_on_docker
def register(parser):
    """
    Register command-line arguments for the ONNX conversion script.
    """
    parser.description = "Convert a PyTorch segmentation model to ONNX format."
    parser.add_argument(
        "--model_path",
        type=str,
        default="results/best_segmentation_model.pth",
        help="Path to the saved PyTorch model (.pth file)",
    )
    parser.add_argument(
        "--output_path",
        type=str,
        default="results/segmentation_model.onnx",
        help="Path where the ONNX model will be saved",
    )
    parser.add_argument("--img_size", type=int, default=512, help="Input image size used during training/export")
    # Add hostname if necessary for your docker setup, similar to optimise_localisation.py
    # parser.add_argument("--hostname", type=str, default="some_default_host", help="Specify the Docker hostname")


@run_on_docker  # Add relevant hostname if needed: @run_on_docker(hostname="some_host")
def run(model_path, output_path, img_size, **kwargs):
    """
    Run the ONNX conversion process.
    """
    convert_to_onnx(model_path, output_path, img_size)
