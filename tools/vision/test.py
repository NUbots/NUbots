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

import matplotlib.pyplot as plt
import numpy as np
import torch
import torchvision.transforms as transforms
from PIL import Image
# Import the model definition
from train import EfficientSegmentationModel

# Assuming run_on_docker is available in your project structure
# If not, you might need to adjust the import or remove the decorator
from utility.dockerise import run_on_docker


def visualize_segmentation(model_path, test_image_path, img_size=512):
    """
    Loads a model, performs segmentation on a test image, and visualizes the result.
    """
    # Parameters are now passed as arguments
    # model_path = "results/best_segmentation_model.pth"
    # test_image_path = "test/images/81-test_nagoya_game_a_00278.png"
    # img_size = 512

    # Define the class colors for visualization
    class_colors = [
        [254, 254, 254],  # Field - #FEFEFE
        [127, 127, 127],  # Field lines - #7F7F7F
        [0, 0, 0],  # Background - #000000
    ]

    # Create results directory if it doesn't exist
    os.makedirs("results", exist_ok=True)

    # Load the model
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    # Updated to use the new model
    model = EfficientSegmentationModel(in_channels=3, num_classes=3)
    model.load_state_dict(torch.load(model_path, map_location=device))
    model.to(device)
    model.eval()

    # Prepare image transformations
    transform = transforms.Compose(
        [
            transforms.Resize((img_size, img_size)),
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
        ]
    )

    # Load and preprocess the test image
    if not os.path.exists(test_image_path):
        print(f"Error: Test image not found at {test_image_path}")
        return

    image = Image.open(test_image_path).convert("RGB")
    original_size = image.size

    # Save original image for display
    original_image = np.array(image)

    # Load ground truth mask if it exists
    test_mask_path = test_image_path.replace("images", "segmentations")
    ground_truth_exists = os.path.exists(test_mask_path)
    if ground_truth_exists:
        mask = Image.open(test_mask_path).convert("RGB")
        mask = mask.resize(original_size, Image.NEAREST)  # Ensure it's the same size
        ground_truth_mask = np.array(mask)
    else:
        ground_truth_mask = np.zeros_like(original_image)
        print(f"Warning: Ground truth mask not found at {test_mask_path}")

    # Transform image for model input
    image_tensor = transform(image).unsqueeze(0).to(device)

    # Run inference
    with torch.no_grad():
        output = model(image_tensor)

    # Process the output
    output = output.squeeze(0).cpu()
    _, predicted = torch.max(output, 0)
    predicted = predicted.numpy()

    # Create colored segmentation map
    segmentation_map = np.zeros((img_size, img_size, 3), dtype=np.uint8)
    for class_idx, color in enumerate(class_colors):
        segmentation_map[predicted == class_idx] = color

    # Resize segmentation map back to original image size if needed
    segmentation_image = Image.fromarray(segmentation_map)
    segmentation_image = segmentation_image.resize(original_size, Image.NEAREST)
    segmentation_map = np.array(segmentation_image)

    # Display the results
    plt.figure(figsize=(18, 6))

    plt.subplot(1, 3, 1)
    plt.title("Original Image")
    plt.imshow(original_image)
    plt.axis("off")

    plt.subplot(1, 3, 2)
    plt.title("Segmentation Result")
    plt.imshow(segmentation_map)
    plt.axis("off")

    plt.subplot(1, 3, 3)
    plt.title("Ground Truth Mask" if ground_truth_exists else "Ground Truth Not Found")
    plt.imshow(ground_truth_mask)
    plt.axis("off")

    plt.tight_layout()
    plt.savefig("results/segmentation_result.png")
    plt.show()

    print("Visualization saved as 'results/segmentation_result.png'")


@run_on_docker
def register(parser):
    """
    Register command-line arguments for the segmentation visualization script.
    """
    parser.description = "Visualize the output of a segmentation model on a test image."
    parser.add_argument(
        "--model_path",
        type=str,
        default="results/best_segmentation_model.pth",
        help="Path to the saved PyTorch model (.pth file)",
    )
    parser.add_argument(
        "--test_image_path",
        type=str,
        required=True,  # Make image path required
        help="Path to the test image file",
    )
    parser.add_argument("--img_size", type=int, default=512, help="Input image size the model expects")
    # Add hostname if necessary for your docker setup
    # parser.add_argument("--hostname", type=str, default="some_default_host", help="Specify the Docker hostname")


@run_on_docker  # Add relevant hostname if needed
def run(model_path, test_image_path, img_size, **kwargs):
    """
    Run the segmentation visualization.
    """
    visualize_segmentation(model_path, test_image_path, img_size)


# Remove the old if __name__ == "__main__": block
# if __name__ == "__main__":
#    visualize_segmentation()
