#!/usr/bin/env python3

import os
from typing import List, Optional, Dict

import numpy as np
import torch
from torch.utils.data import Dataset
import yaml


class WalkDataset(Dataset):
    def __init__(self, file_paths: List[str], metadata: Dict, stats: Optional[Dict] = None):
        """
        Loads the binary walk data from a list of files.
        
        Args:
            file_paths: List of paths to .bin episode files
            metadata: The metadata dictionary loaded from metadata.yaml
            stats: Optional dictionary containing 'obs_mean', 'obs_std', 'target_mean', 'target_std'
                   for normalization. If not provided, data will not be normalized.
        """
        self.file_paths = file_paths
        self.metadata = metadata
        self.stats = stats
        
        self.obs_dim = self.metadata["obs_dim"]
        self.target_dim = self.metadata["target_dim"]
        self.sample_dim = self.metadata["sample_dim"]
        
        # We will load the specified files into memory
        print(f"Loading {len(self.file_paths)} episodes into memory...")
        
        data_list = []
        for file_path in self.file_paths:
            data = np.fromfile(file_path, dtype=np.float32).reshape(-1, self.sample_dim)
            data_list.append(data)
            
        self.all_data = np.concatenate(data_list, axis=0)
        
        self.observations = self.all_data[:, :self.obs_dim]
        self.targets = self.all_data[:, self.obs_dim:]
        
        print(f"Loaded {len(self.all_data)} samples.")

        # Normalize data if stats are provided
        if self.stats is not None:
            print("Applying normalization...")
            self.obs_mean = self.stats["obs_mean"]
            self.obs_std = self.stats["obs_std"]
            self.target_mean = self.stats["target_mean"]
            self.target_std = self.stats["target_std"]
            
            # Avoid division by zero
            self.obs_std[self.obs_std < 1e-8] = 1.0
            self.target_std[self.target_std < 1e-8] = 1.0
            
            self.observations = (self.observations - self.obs_mean) / self.obs_std
            self.targets = (self.targets - self.target_mean) / self.target_std

    def __len__(self):
        return len(self.all_data)

    def __getitem__(self, idx):
        obs = torch.from_numpy(self.observations[idx])
        target = torch.from_numpy(self.targets[idx])
        return obs, target

def compute_dataset_statistics(file_paths: List[str], sample_dim: int, obs_dim: int):
    """
    Computes mean and std deviation over a list of episode files.
    """
    print(f"Computing statistics over {len(file_paths)} files...")
    data_list = []
    for file_path in file_paths:
        data = np.fromfile(file_path, dtype=np.float32).reshape(-1, sample_dim)
        data_list.append(data)
    all_data = np.concatenate(data_list, axis=0)
    
    observations = all_data[:, :obs_dim]
    targets = all_data[:, obs_dim:]
    
    stats = {
        "obs_mean": np.mean(observations, axis=0, dtype=np.float32),
        "obs_std": np.std(observations, axis=0, dtype=np.float32),
        "target_mean": np.mean(targets, axis=0, dtype=np.float32),
        "target_std": np.std(targets, axis=0, dtype=np.float32),
    }
    return stats
