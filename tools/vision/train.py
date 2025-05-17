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
import random

import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from PIL import Image
from torch.optim.lr_scheduler import ReduceLROnPlateau
from torch.utils.data import DataLoader, Dataset
from torchvision import transforms
from tqdm import tqdm

# Assuming run_on_docker is available in your project structure
# If not, you might need to adjust the import or remove the decorator
from utility.dockerise import run_on_docker

from .dataset import SegmentationDataset
from .efficient_segmentation_model import EfficientSegmentationModel
from .lightweight_segmentation_model import LightSegmentationModel


# Improved training function with learning rate scheduling
def train_model(model, train_loader, val_loader, criterion, optimizer, device, num_epochs=20, best_model_save_path="results/best_segmentation_model.pth"):
    best_loss = float("inf")

    # Add mixed precision training
    scaler = torch.cuda.amp.GradScaler()

    # Add learning rate scheduler for better convergence
    scheduler = ReduceLROnPlateau(optimizer, mode="min", factor=0.5, patience=3)

    # Ensure the directory for saving the model exists
    os.makedirs(os.path.dirname(best_model_save_path), exist_ok=True)

    for epoch in range(num_epochs):
        # Training
        model.train()
        train_loss = 0.0

        for images, masks in tqdm(train_loader, desc=f"Epoch {epoch + 1}/{num_epochs} - Training"):
            images = images.to(device)
            masks = masks.to(device)

            optimizer.zero_grad()

            # Use mixed precision training
            with torch.cuda.amp.autocast():
                outputs = model(images)
                loss = criterion(outputs, masks)

            # Scale loss and backpropagate
            scaler.scale(loss).backward()
            scaler.step(optimizer)
            scaler.update()

            train_loss += loss.item() * images.size(0)

        train_loss = train_loss / len(train_loader.dataset)

        # Validation
        model.eval()
        val_loss = 0.0
        correct_pixels = 0
        total_pixels = 0

        with torch.no_grad():
            for images, masks in tqdm(val_loader, desc=f"Epoch {epoch + 1}/{num_epochs} - Validation"):
                images = images.to(device)
                masks = masks.to(device)

                outputs = model(images)
                loss = criterion(outputs, masks)
                val_loss += loss.item() * images.size(0)

                # Calculate pixel accuracy
                preds = torch.argmax(outputs, dim=1)
                correct_pixels += (preds == masks).sum().item()
                total_pixels += masks.numel()

        val_loss = val_loss / len(val_loader.dataset)
        pixel_acc = 100 * correct_pixels / total_pixels

        # Update learning rate based on validation loss
        scheduler.step(val_loss)

        print(
            f"Epoch {epoch + 1}/{num_epochs}: "
            f"Train Loss: {train_loss:.4f}, Val Loss: {val_loss:.4f}, "
            f"Pixel Acc: {pixel_acc:.2f}%"
        )

        # Save best model
        if val_loss < best_loss:
            best_loss = val_loss
            torch.save(model.state_dict(), best_model_save_path)
            print(f"Model saved to {best_model_save_path} with validation loss: {val_loss:.4f}")


@run_on_docker
def register(parser):
    """
    Register command-line arguments for the training script.
    """
    parser.description = "Train a segmentation model."
    parser.add_argument("--train_img_dir", type=str, default="datasets/webots/train/images", help="Directory for training images")
    parser.add_argument(
        "--train_mask_dir", type=str, default="datasets/webots/train/segmentations", help="Directory for training masks"
    )
    parser.add_argument("--test_img_dir", type=str, default="datasets/webots/test/images", help="Directory for validation images")
    parser.add_argument(
        "--test_mask_dir", type=str, default="datasets/webots/test/segmentations", help="Directory for validation masks"
    )
    parser.add_argument("--img_size", type=int, default=512, help="Image size for training and validation")
    parser.add_argument("--batch_size", type=int, default=8, help="Training batch size")
    parser.add_argument("--num_epochs", type=int, default=10, help="Number of training epochs")
    parser.add_argument("--lr", type=float, default=0.001, help="Initial learning rate")
    parser.add_argument("--weight_decay", type=float, default=1e-4, help="Optimizer weight decay")
    parser.add_argument("--num_workers", type=int, default=2, help="Number of workers for DataLoader")
    parser.add_argument("--results_dir", type=str, default="recordings", help="Directory to save results (model, etc.)")
    parser.add_argument(
        "--export_onnx", action="store_true", help="Export the final model to ONNX format after training"
    )
    # parser.add_argument("--shm_size", type=str, default="16G", help="Shared memory size for training")
    parser.add_argument(
        "--model_type",
        type=str,
        default="lightweight",
        choices=["efficient", "lightweight"],
        help="Type of model to train (efficient, lightweight)",
    )

@run_on_docker  # Add relevant hostname if needed
def run(
    train_img_dir,
    train_mask_dir,
    test_img_dir,
    test_mask_dir,
    img_size,
    batch_size,
    num_epochs,
    lr,
    weight_decay,
    num_workers,
    results_dir,
    export_onnx,
    model_type,
    **kwargs,
):
    """
    Run the model training process.
    """
    # Create results directory if it doesn't exist
    os.makedirs(results_dir, exist_ok=True)
    best_model_path = os.path.join(results_dir, "best_segmentation_model.pth")
    onnx_output_path = os.path.join(results_dir, "segmentation_model.onnx")

    # Set device
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"Using device: {device}")

    # Image transformations
    transform = transforms.Compose(
        [
            # Resize is now done within the Dataset
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
        ]
    )

    # Create datasets with consistent image size and augmentation for training
    train_dataset = SegmentationDataset(
        train_img_dir,
        train_mask_dir,
        transform=transform,
        img_size=img_size,
        augment=True,
    )
    test_dataset = SegmentationDataset(
        test_img_dir,
        test_mask_dir,
        transform=transform,
        img_size=img_size,
        augment=False,
    )

    # Create dataloaders with pin_memory=True
    train_loader = DataLoader(
        train_dataset,
        batch_size=batch_size,
        shuffle=True,
        num_workers=num_workers,
        pin_memory=True
    )
    test_loader = DataLoader(
        test_dataset,
        batch_size=batch_size,
        shuffle=False,
        num_workers=num_workers,
        pin_memory=True
    )

    # Initialize model based on model_type
    if model_type.lower() == "lightweight":
        model = LightSegmentationModel(in_channels=3, num_classes=3)
        print("Using Lightweight Segmentation Model")
    else:
        model = EfficientSegmentationModel(num_classes=3)
        print("Using Efficient Segmentation Model")

    model = model.to(device)

    # Loss function for multi-class segmentation
    # Add class weighting if dataset is imbalanced
    criterion = nn.CrossEntropyLoss()
    optimizer = optim.Adam(model.parameters(), lr=lr, weight_decay=weight_decay)

    # Train model (calling the updated version)
    print(f"Starting training for {num_epochs} epochs...")
    train_model(model, train_loader, test_loader, criterion, optimizer, device, num_epochs, best_model_path)
    print(f"Training finished. Best model saved to {best_model_path}")

    # Export to ONNX if requested
    if export_onnx:
        print(f"Exporting model to ONNX format at {onnx_output_path}...")
        # Load the best model state for export
        try:
            model.load_state_dict(torch.load(best_model_path, map_location=device))
            model.eval() # Ensure model is in eval mode for export

            dummy_input = torch.randn(1, 3, img_size, img_size).to(device)
            torch.onnx.export(
                model,
                dummy_input,
                onnx_output_path,
                export_params=True,
                opset_version=11,
                input_names=["input"],
                output_names=["output"],
                dynamic_axes={"input": {0: "batch_size"}, "output": {0: "batch_size"}},
            )
            print(f"Model successfully exported to {onnx_output_path}")
        except FileNotFoundError:
            print(f"Error: Could not find best model at {best_model_path} to export to ONNX.")
        except Exception as e:
            print(f"An error occurred during ONNX export: {e}")
