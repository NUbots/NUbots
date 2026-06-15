#!/usr/bin/env python3

import torch
import torch.nn as nn


class WalkPolicy(nn.Module):
    def __init__(self, obs_dim: int = 46, target_dim: int = 12):
        """
        MLP Policy mapping walk engine observations to joint angles.
        
        Architecture:
        Input -> LayerNorm -> Linear(256) -> ELU -> Linear(256) -> ELU -> Linear(128) -> ELU -> Linear(12)
        """
        super().__init__()
        
        self.net = nn.Sequential(
            nn.LayerNorm(obs_dim),
            nn.Linear(obs_dim, 256),
            nn.ELU(),
            nn.Linear(256, 256),
            nn.ELU(),
            nn.Linear(256, 128),
            nn.ELU(),
            nn.Linear(128, target_dim)
        )

    def forward(self, obs: torch.Tensor) -> torch.Tensor:
        """
        Args:
            obs: Tensor of shape (..., obs_dim)
        Returns:
            targets: Tensor of shape (..., target_dim) representing the 12 leg joint angles
        """
        return self.net(obs)
