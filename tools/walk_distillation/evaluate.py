#!/usr/bin/env python3

import argparse
import glob
import os

import numpy as np
import torch
import yaml
from tqdm import tqdm

from .model import WalkPolicy

def register(command):
    command.description = "Evaluate the trained policy in a closed-loop rollout against a ground-truth episode"

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
        "--data-dir", "-d",
        type=str,
        default="recordings/walk_data",
        help="Directory containing metadata.yaml and episode bins"
    )


def run(checkpoint, stats, data_dir, **kwargs):
    print(f"Loading checkpoint from {checkpoint}...")
    
    with open(os.path.join(data_dir, "metadata.yaml"), "r") as f:
        metadata = yaml.safe_load(f)
        
    obs_dim = metadata["obs_dim"]
    target_dim = metadata["target_dim"]
    sample_dim = metadata["sample_dim"]
    dt = metadata["dt"]
    step_period = metadata["step_period"]
    
    # Load model
    model = WalkPolicy(obs_dim=obs_dim, target_dim=target_dim)
    checkpoint_data = torch.load(checkpoint, map_location="cpu")
    if "model_state_dict" in checkpoint_data:
        model.load_state_dict(checkpoint_data["model_state_dict"])
    else:
        model.load_state_dict(checkpoint_data)
    model.eval()
    
    print(f"Loading statistics from {stats}...")
    np_stats = np.load(stats)
    obs_mean = np_stats["obs_mean"]
    obs_std = np_stats["obs_std"]
    target_mean = np_stats["target_mean"]
    target_std = np_stats["target_std"]
    
    obs_std[obs_std < 1e-8] = 1.0
    target_std[target_std < 1e-8] = 1.0
    
    # Grab the last episode file as validation
    all_files = sorted(glob.glob(os.path.join(data_dir, "episode_*.bin")))
    if not all_files:
        print("No episode files found!")
        return
        
    val_file = all_files[-1]
    print(f"Running closed-loop validation on ground-truth episode: {val_file}")
    
    # Load episode data
    data = np.fromfile(val_file, dtype=np.float32).reshape(-1, sample_dim)
    gt_observations = data[:, :obs_dim]
    gt_targets = data[:, obs_dim:]
    timesteps = len(data)
    
    # We will simulate a forward pass where the model's outputs become its future inputs.
    # Initial history is drawn directly from the first frame's ground truth history to warm start
    # The action history is stored in the observation at indices 10 to 45
    # obs = [cmd(3), clock(2), phase(1), engine(4), hist1(12), hist2(12), hist3(12)]
    action_history = np.zeros((3, target_dim), dtype=np.float32)
    action_history[0] = gt_observations[0, 10:22]
    action_history[1] = gt_observations[0, 22:34]
    action_history[2] = gt_observations[0, 34:46]
    
    predictions = []
    
    with torch.no_grad():
        for t in range(timesteps):
            gt_obs = gt_observations[t]
            
            # The environment variables (commands, clock, phase, engine state) are driven by the walk engine, 
            # so we take them directly from the ground truth observation for this timestep.
            # Only the action history is replaced by our closed-loop autoregressive predictions!
            
            current_velocity = gt_obs[0:3]
            phase_clock = gt_obs[3:5]
            phase_indicator = gt_obs[5:6]
            engine_state = gt_obs[6:10]
            
            # Construct the autoregressive observation
            obs = np.concatenate([
                current_velocity,
                phase_clock,
                phase_indicator,
                engine_state,
                action_history[0], # t-1
                action_history[1], # t-2
                action_history[2], # t-3
            ])
            
            # Normalize observation
            norm_obs = (obs - obs_mean) / obs_std
            obs_tensor = torch.from_numpy(norm_obs).float().unsqueeze(0)
            
            # Predict
            norm_pred = model(obs_tensor).squeeze(0).numpy()
            
            # Denormalize
            pred = (norm_pred * target_std) + target_mean
            predictions.append(pred)
            
            # Update history (shift)
            action_history[2] = action_history[1]
            action_history[1] = action_history[0]
            action_history[0] = pred

    predictions = np.array(predictions)
    
    # Compare with ground truth
    mae_per_joint = np.mean(np.abs(predictions - gt_targets), axis=0)
    overall_mae = np.mean(mae_per_joint)
    
    print("\nMean Absolute Error (MAE) per joint (in radians):")
    for i, mae in enumerate(mae_per_joint):
        print(f"  Joint {i:2d}: {mae:.4f} rad ({(mae * 180 / np.pi):.2f} deg)")
        
    print(f"\nOverall Rollout MAE: {overall_mae:.4f} rad ({(overall_mae * 180 / np.pi):.2f} deg)")
    
    if overall_mae < 0.05:
        print("Success: The policy accurately tracks the ground-truth trajectory in closed loop!")
    else:
        print("Warning: The policy accumulated significant drift during the closed-loop rollout.")
