#!/usr/bin/env python3

import argparse
import glob
import os

import numpy as np
import torch
import torch.nn as nn
from torch.utils.data import DataLoader
from torch.utils.tensorboard import SummaryWriter
from tqdm import tqdm
import yaml

from .dataset import WalkDataset, compute_dataset_statistics
from .model import WalkPolicy

def register(command):
    command.description = "Train the walk distillation policy"

    command.add_argument(
        "--data-dir", "-d",
        type=str,
        default="recordings/walk_data",
        help="Directory containing the training data (.bin files and metadata.yaml)"
    )
    command.add_argument(
        "--output-dir", "-o",
        type=str,
        default="checkpoints",
        help="Directory to save the trained models"
    )
    command.add_argument(
        "--batch-size", "-b",
        type=int,
        default=2048,
        help="Batch size"
    )
    command.add_argument(
        "--epochs", "-e",
        type=int,
        default=200,
        help="Number of epochs to train"
    )
    command.add_argument(
        "--lr",
        type=float,
        default=1e-3,
        help="Learning rate"
    )


def run(data_dir, output_dir, batch_size, epochs, lr, **kwargs):
    os.makedirs(output_dir, exist_ok=True)

    # Load metadata
    with open(os.path.join(data_dir, "metadata.yaml"), "r") as f:
        metadata = yaml.safe_load(f)

    # Find all episode files
    all_files = sorted(glob.glob(os.path.join(data_dir, "episode_*.bin")))
    if not all_files:
        print(f"No binary files found in {data_dir}")
        return

    # Split into train and val (90% train, 10% val)
    # Splitting by file ensures no data leakage between contiguous timesteps
    split_idx = int(len(all_files) * 0.9)
    train_files = all_files[:split_idx]
    val_files = all_files[split_idx:]

    print(f"Found {len(all_files)} episodes. Split: {len(train_files)} train, {len(val_files)} val.")

    # Compute statistics on train set ONLY
    stats = compute_dataset_statistics(train_files, metadata["sample_dim"], metadata["obs_dim"])
    
    # Save statistics for later inference/export
    stats_path = os.path.join(output_dir, "dataset_stats.npz")
    np.savez(stats_path, **stats)
    print(f"Saved dataset statistics to {stats_path}")

    print("Loading datasets...")
    train_dataset = WalkDataset(train_files, metadata, stats=stats)
    val_dataset = WalkDataset(val_files, metadata, stats=stats)

    train_loader = DataLoader(train_dataset, batch_size=batch_size, shuffle=True, num_workers=4, pin_memory=True)
    val_loader = DataLoader(val_dataset, batch_size=batch_size, shuffle=False, num_workers=4, pin_memory=True)

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"Using device: {device}")

    model = WalkPolicy(obs_dim=metadata["obs_dim"], target_dim=metadata["target_dim"]).to(device)
    optimizer = torch.optim.Adam(model.parameters(), lr=lr, weight_decay=1e-5)
    scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(optimizer, T_max=epochs)
    
    # Using Huber loss (SmoothL1) which is more robust to outliers than MSE
    criterion = nn.SmoothL1Loss()

    best_val_loss = float("inf")
    
    # Initialize TensorBoard writer
    log_dir = os.path.join(output_dir, "logs")
    writer = SummaryWriter(log_dir=log_dir)
    print(f"TensorBoard logging to {log_dir}")

    for epoch in range(1, epochs + 1):
        model.train()
        train_loss = 0.0
        
        pbar = tqdm(train_loader, desc=f"Epoch {epoch}/{epochs} [Train]")
        for obs, target in pbar:
            obs, target = obs.to(device), target.to(device)
            
            # Inject noise into the history frames (indices 10 to 45) to combat autoregressive exposure bias.
            # This teaches the network to recover from its own slight prediction errors during closed-loop rollout,
            # completely eliminating the drift without needing hardcoded EMA hacks in C++.
            noise = torch.randn_like(obs[:, 10:46]) * 0.05
            obs[:, 10:46] += noise
            
            optimizer.zero_grad()
            pred = model(obs)
            loss = criterion(pred, target)
            loss.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(), 1.0)
            optimizer.step()
            
            train_loss += loss.item()
            pbar.set_postfix({"loss": f"{loss.item():.5f}"})
            
        train_loss /= len(train_loader)
        
        # Validation
        model.eval()
        val_loss = 0.0
        with torch.no_grad():
            for obs, target in tqdm(val_loader, desc=f"Epoch {epoch}/{epochs} [Val]"):
                obs, target = obs.to(device), target.to(device)
                pred = model(obs)
                loss = criterion(pred, target)
                val_loss += loss.item()
                
        val_loss /= len(val_loader)
        
        # Log to TensorBoard
        writer.add_scalar('Loss/train', train_loss, epoch)
        writer.add_scalar('Loss/val', val_loss, epoch)
        writer.add_scalar('LearningRate', scheduler.get_last_lr()[0], epoch)
        
        scheduler.step()
        
        print(f"Epoch {epoch}: Train Loss = {train_loss:.6f}, Val Loss = {val_loss:.6f}")
        
        if val_loss < best_val_loss:
            best_val_loss = val_loss
            # Save the model state and optimizer state
            checkpoint = {
                "epoch": epoch,
                "model_state_dict": model.state_dict(),
                "optimizer_state_dict": optimizer.state_dict(),
                "val_loss": val_loss
            }
            save_path = os.path.join(output_dir, "best_model.pth")
            torch.save(checkpoint, save_path)
            print(f"Saved new best model to {save_path}")

    writer.close()
    print("Training complete.")
