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

class ActorCritic(nn.Module):
    def __init__(self, num_obs, num_actions):
        super().__init__()
        
        # Policy (Actor) outputs student residuals
        self.actor = nn.Sequential(
            nn.Linear(num_obs, 256),
            nn.ELU(),
            nn.Linear(256, 128),
            nn.ELU(),
            nn.Linear(128, 64),
            nn.ELU(),
            nn.Linear(64, num_actions)
        )
        # Initialize actor to output ~zero residuals initially (zero weights on last layer)
        nn.init.zeros_(self.actor[-1].weight)
        nn.init.zeros_(self.actor[-1].bias)
        
        # Value Function (Critic)
        self.critic = nn.Sequential(
            nn.Linear(num_obs, 256),
            nn.ELU(),
            nn.Linear(256, 128),
            nn.ELU(),
            nn.Linear(128, 64),
            nn.ELU(),
            nn.Linear(64, 1)
        )
        
        # Action standard deviation
        self.std = nn.Parameter(torch.zeros(num_actions))
        
        self.register_buffer("obs_mean", torch.zeros(num_obs))
        self.register_buffer("obs_var", torch.ones(num_obs))
        self.register_buffer("obs_count", torch.tensor(1e-4))
        self.clip_obs = 5.0
        
    def update_obs_norm(self, obs):
        valid = ~torch.isnan(obs).any(dim=1) & ~torch.isinf(obs).any(dim=1)
        valid_obs = obs[valid]
        if valid_obs.shape[0] == 0:
            return
            
        batch_mean = valid_obs.mean(dim=0)
        batch_var = valid_obs.var(dim=0, unbiased=False)
        batch_count = valid_obs.shape[0]
        
        delta = batch_mean - self.obs_mean
        tot_count = self.obs_count + batch_count
        
        new_mean = self.obs_mean + delta * batch_count / tot_count
        m_a = self.obs_var * self.obs_count
        m_b = batch_var * batch_count
        M2 = m_a + m_b + torch.square(delta) * self.obs_count * batch_count / tot_count
        new_var = M2 / tot_count
        
        self.obs_mean.copy_(new_mean)
        self.obs_var.copy_(new_var)
        self.obs_count.copy_(tot_count)

    def normalize_obs(self, obs):
        return torch.clamp((obs - self.obs_mean) / torch.sqrt(self.obs_var + 1e-8), -self.clip_obs, self.clip_obs)

    def forward(self, obs):
        obs_norm = self.normalize_obs(obs)
        mean = self.actor(obs_norm)
        std = self.std.exp()
        return mean, std
    
    def evaluate(self, obs, action):
        obs_norm = self.normalize_obs(obs)
        mean = self.actor(obs_norm)
        std = self.std.exp()
        dist = torch.distributions.Normal(mean, std)
        log_prob = dist.log_prob(action).sum(dim=-1)
        entropy = dist.entropy().sum(dim=-1)
        value = self.critic(obs_norm).squeeze(-1)
        return log_prob, value, entropy

    def act(self, obs, update_norm=True):
        if update_norm:
            self.update_obs_norm(obs)
        obs_norm = self.normalize_obs(obs)
        mean = self.actor(obs_norm)
        std = self.std.exp()
        dist = torch.distributions.Normal(mean, std)
        action = dist.sample()
        log_prob = dist.log_prob(action).sum(dim=-1)
        value = self.critic(obs_norm).squeeze(-1)
        return action, log_prob, value
