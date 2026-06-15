#!/usr/bin/env python3

import os

import numpy as np
import torch
import torch.nn as nn

from .model import WalkPolicy

class NormalizedWalkPolicy(nn.Module):
    def __init__(self, policy: WalkPolicy, obs_mean: np.ndarray, obs_std: np.ndarray, target_mean: np.ndarray, target_std: np.ndarray):
        """
        Wraps the core MLP policy with normalization constants so that the ONNX
        model expects raw observations and outputs raw joint angles.
        """
        super().__init__()
        self.policy = policy
        
        # Avoid division by zero
        obs_std[obs_std < 1e-8] = 1.0
        target_std[target_std < 1e-8] = 1.0
        
        # Register buffers so they are part of the model but not updated by gradients
        self.register_buffer("obs_mean", torch.from_numpy(obs_mean).float())
        self.register_buffer("obs_std", torch.from_numpy(obs_std).float())
        self.register_buffer("target_mean", torch.from_numpy(target_mean).float())
        self.register_buffer("target_std", torch.from_numpy(target_std).float())

    def forward(self, raw_obs: torch.Tensor) -> torch.Tensor:
        # 1. Normalize input
        norm_obs = (raw_obs - self.obs_mean) / self.obs_std
        
        # 2. Forward pass
        norm_target = self.policy(norm_obs)
        
        # 3. Denormalize output
        raw_target = (norm_target * self.target_std) + self.target_mean
        
        return raw_target


def register(command):
    command.description = "Export trained walk distillation policy to ONNX, baking in normalization"

    command.add_argument(
        "--checkpoint", "-c",
        type=str,
        required=True,
        help="Path to the trained model checkpoint (.pth)"
    )
    command.add_argument(
        "--stats", "-s",
        type=str,
        default="checkpoints/dataset_stats.npz",
        help="Path to the dataset statistics (.npz)"
    )
    command.add_argument(
        "--output", "-o",
        type=str,
        default="walk_policy.onnx",
        help="Path to save the ONNX model"
    )
    command.add_argument(
        "--obs-dim",
        type=int,
        default=46,
        help="Observation dimension"
    )
    command.add_argument(
        "--target-dim",
        type=int,
        default=12,
        help="Target dimension"
    )


def run(checkpoint, stats, output, obs_dim, target_dim, **kwargs):
    print(f"Loading checkpoint from {checkpoint}...")
    
    # Load model
    model = WalkPolicy(obs_dim=obs_dim, target_dim=target_dim)
    
    # Extract the state dict depending on how it was saved
    checkpoint_data = torch.load(checkpoint, map_location="cpu")
    if "model_state_dict" in checkpoint_data:
        model.load_state_dict(checkpoint_data["model_state_dict"])
    else:
        model.load_state_dict(checkpoint_data)
        
    model.eval()
    
    print(f"Loading statistics from {stats}...")
    np_stats = np.load(stats)
    
    # Wrap model
    wrapped_model = NormalizedWalkPolicy(
        model, 
        np_stats["obs_mean"], 
        np_stats["obs_std"], 
        np_stats["target_mean"], 
        np_stats["target_std"]
    )
    wrapped_model.eval()
    
    print(f"Exporting to {output}...")
    
    dummy_input = torch.randn(1, obs_dim)
    
    torch.onnx.export(
        wrapped_model,
        dummy_input,
        output,
        input_names=["observation"],
        output_names=["joint_targets"],
        dynamic_axes={"observation": {0: "batch"}, "joint_targets": {0: "batch"}},
        opset_version=17,
    )
    
    print("Export complete. The ONNX model now accepts unnormalized inputs and provides unnormalized targets.")
