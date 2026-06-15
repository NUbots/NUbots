import os
import sys
import time
import torch
import torch.nn as nn
import torch.optim as optim
import numpy as np
from torch.utils.tensorboard import SummaryWriter

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(HERE, "..", ".."))

from tools.walk_distillation.env import NugusEnv
from tools.walk_distillation.model import WalkPolicy, ActorCritic



class RolloutBuffer:
    def __init__(self, num_envs, num_steps, num_obs, num_actions, device):
        self.obs = torch.zeros((num_steps, num_envs, num_obs), device=device)
        self.actions = torch.zeros((num_steps, num_envs, num_actions), device=device)
        self.log_probs = torch.zeros((num_steps, num_envs), device=device)
        self.rewards = torch.zeros((num_steps, num_envs), device=device)
        self.dones = torch.zeros((num_steps, num_envs), device=device)
        self.values = torch.zeros((num_steps, num_envs), device=device)
        self.step = 0

    def add(self, obs, action, log_prob, reward, done, value):
        self.obs[self.step] = obs
        self.actions[self.step] = action
        self.log_probs[self.step] = log_prob
        self.rewards[self.step] = reward
        self.dones[self.step] = done
        self.values[self.step] = value
        self.step += 1

    def compute_returns(self, next_value, next_done, gamma=0.99, gae_lambda=0.95):
        returns = torch.zeros_like(self.rewards)
        advs = torch.zeros_like(self.rewards)
        last_gaelam = 0
        
        for t in reversed(range(self.step)):
            if t == self.step - 1:
                next_non_terminal = 1.0 - next_done.float()
                next_values = next_value
            else:
                next_non_terminal = 1.0 - self.dones[t + 1].float()
                next_values = self.values[t + 1]
                
            delta = self.rewards[t] + gamma * next_values * next_non_terminal - self.values[t]
            advs[t] = last_gaelam = delta + gamma * gae_lambda * next_non_terminal * last_gaelam
            
        returns = advs + self.values
        return returns, advs

def compute_reward_and_done(env, obs, actions, prev_actions):
    commands = env.commands
    lin_vel = obs[:, 6:9]
    ang_vel = obs[:, 9:12]
    proj_gravity = obs[:, 3:6]
    
    # Velocity Tracking
    xy_error = torch.sum(torch.square(commands[:, :2] - lin_vel[:, :2]), dim=1)
    yaw_error = torch.square(commands[:, 2] - ang_vel[:, 2])
    rew_lin_vel = torch.exp(-xy_error / 0.25)
    rew_ang_vel = torch.exp(-yaw_error / 0.5)
    
    # Posture/Orientation
    orient_error = torch.sum(torch.square(proj_gravity[:, :2]), dim=1)
    rew_orient = torch.exp(-orient_error / 0.2)
    
    # Action Smoothness
    rew_action_rate = torch.sum(torch.square(actions - prev_actions), dim=1)
    
    # Survival bonus
    rew_survival = torch.ones_like(rew_orient)
    
    reward = (
        1.0 * rew_lin_vel + 
        0.5 * rew_ang_vel + 
        1.0 * rew_orient +
        1.0 * rew_survival -
        0.01 * rew_action_rate
    )
    
    # Replace any NaNs in reward with a penalty
    invalid_mask = torch.isnan(reward) | torch.isinf(reward) | torch.isnan(obs).any(dim=-1) | torch.isinf(obs).any(dim=-1)
    reward[invalid_mask] = -10.0
    
    # Termination (Tilt > 45 degrees or NaN/Inf state)
    limit_angle = 45.0 * 3.14159 / 180.0
    tilt = torch.acos(torch.clamp(-proj_gravity[:, 2], -1.0, 1.0)).abs()
    invalid_state = torch.isnan(obs).any(dim=-1) | torch.isinf(obs).any(dim=-1)
    done = (tilt > limit_angle) | invalid_state
    done = done.to(torch.uint8)
    
    metrics = {
        "reward/total": reward.mean().item(),
        "reward/lin_vel": rew_lin_vel.mean().item(),
        "reward/ang_vel": rew_ang_vel.mean().item(),
        "reward/orient": rew_orient.mean().item(),
        "reward/survival": rew_survival.mean().item(),
        "reward/action_rate": -0.01 * rew_action_rate.mean().item(),
    }
    
    return reward, done, metrics

def resample_commands(env, env_ids):
    if len(env_ids) == 0:
        return
    env.commands[env_ids, 0] = torch.rand(len(env_ids), device=env.device) * 0.3 - 0.15 # vx
    env.commands[env_ids, 1] = torch.rand(len(env_ids), device=env.device) * 0.2 - 0.1  # vy
    env.commands[env_ids, 2] = torch.rand(len(env_ids), device=env.device) * 0.5 - 0.25 # vtheta

def register(command):
    command.help = "Train the residual RL policy using mjwarp"
    command.add_argument("--checkpoint", "-c", type=str, default="checkpoints/best_model.pth", help="Teacher policy checkpoint")
    command.add_argument("--stats", "-s", type=str, default="checkpoints/dataset_stats.npz", help="Dataset stats file")

def run(checkpoint, stats, **kwargs):
    device = torch.device('cuda:0' if torch.cuda.is_available() else 'cpu')
    print(f"Training on device: {device}")
    
    # Load stats
    stats_data = np.load(stats)
    teacher_obs_mean = torch.from_numpy(stats_data["obs_mean"]).float().to(device)
    teacher_obs_std = torch.from_numpy(stats_data["obs_std"]).float().to(device)
    teacher_target_mean = torch.from_numpy(stats_data["target_mean"]).float().to(device)
    teacher_target_std = torch.from_numpy(stats_data["target_std"]).float().to(device)
    
    teacher_obs_std[teacher_obs_std < 1e-8] = 1.0
    teacher_target_std[teacher_target_std < 1e-8] = 1.0
    
    # Load teacher
    teacher_model = WalkPolicy(obs_dim=len(teacher_obs_mean), target_dim=12).to(device)
    checkpoint_data = torch.load(checkpoint, map_location=device)
    teacher_model.load_state_dict(checkpoint_data.get("model_state_dict", checkpoint_data))
    teacher_model.eval()
    for param in teacher_model.parameters():
        param.requires_grad = False
    
    # PPO Hyperparams
    num_envs = 4096
    num_steps = 24       # steps per rollout
    epochs = 5
    batch_size = num_envs * num_steps
    mini_batch_size = batch_size // 4
    clip_param = 0.2
    max_grad_norm = 1.0
    lr = 1e-3
    desired_kl = 0.01
    iterations = 5000
    
    # Initialize env
    env = NugusEnv(nenv=num_envs, device=device)
    env.reset_all()
    resample_commands(env, torch.arange(num_envs, device=device))
    
    # RL network
    # Obs shape: cmds(3) + proj_g(3) + lin_v(3) + ang_v(3) + qpos_err(40) + qvel(40) + last_actions(20) = 112
    # But last_actions is size NU=20? Actually in mjwarp env, qpos is nq=47, qvel is nv=46, ctrl is nu=40.
    # We should let the env dynamically tell us the sizes.
    obs_dim = env.get_obs().shape[1]
    action_dim = 12 # We only residualize the legs (12 joints)
    
    ac = ActorCritic(num_obs=obs_dim, num_actions=action_dim).to(device)
    optimizer = optim.Adam(ac.parameters(), lr=lr)
    
    buffer = RolloutBuffer(num_envs, num_steps, obs_dim, action_dim, device)
    
    # Initialize TensorBoard writer
    writer = SummaryWriter(log_dir="checkpoints/logs/rl")
    
    obs = env.get_obs()
    
    env_episode_length = torch.zeros(num_envs, dtype=torch.int32, device=device)
    max_episode_length = 500
    
    for it in range(iterations):
        t0 = time.time()
        
        buffer.step = 0
        total_reward = 0.0
        
        # Accumulate metrics over the rollout
        epoch_metrics = {
            "reward/total": 0.0,
            "reward/lin_vel": 0.0,
            "reward/ang_vel": 0.0,
            "reward/orient": 0.0,
            "reward/survival": 0.0,
            "reward/action_rate": 0.0,
        }
        
        prev_actions = torch.zeros((num_envs, action_dim), device=device)
        
        for step in range(num_steps):
            # 1. Get teacher targets
            teacher_obs = env.get_teacher_obs()
            norm_teacher_obs = (teacher_obs - teacher_obs_mean) / teacher_obs_std
            with torch.no_grad():
                norm_teacher_target = teacher_model(norm_teacher_obs)
            teacher_targets = (norm_teacher_target * teacher_target_std) + teacher_target_mean
            
            # 2. Get student residuals
            with torch.no_grad():
                actions, log_probs, values = ac.act(obs)
                
            # 3. Step env
            env.step(teacher_targets, actions, alpha=0.2)
            next_obs = env.get_obs()
            
            # 4. Rewards
            rewards, dones, step_metrics = compute_reward_and_done(env, next_obs, actions, prev_actions)
            prev_actions = actions.clone()
            
            # Accumulate metrics
            total_reward += rewards.mean().item()
            for k in epoch_metrics:
                epoch_metrics[k] += step_metrics[k]
            
            env_episode_length += 1
            timeouts = (env_episode_length >= max_episode_length).to(torch.uint8)
            resets = dones | timeouts
            
            if resets.any():
                reset_indices = resets.nonzero(as_tuple=True)[0]
                env.reset_done(resets)
                resample_commands(env, reset_indices)
                env_episode_length[reset_indices] = 0
                prev_actions[reset_indices] = 0.0
                next_obs = env.get_obs() # recompute obs after reset
                
            buffer.add(obs, actions, log_probs, rewards, dones, values)
            obs = next_obs
            
        # PPO Update
        with torch.no_grad():
            _, _, next_values = ac.act(obs, update_norm=False)
            returns, advs = buffer.compute_returns(next_values, dones)
            advs = (advs - advs.mean()) / (advs.std() + 1e-8)
            
        b_obs = buffer.obs.reshape(-1, obs_dim)
        b_actions = buffer.actions.reshape(-1, action_dim)
        b_log_probs = buffer.log_probs.reshape(-1)
        b_returns = returns.reshape(-1)
        b_advs = advs.reshape(-1)
        b_values = buffer.values.reshape(-1)
        
        mean_kl = 0.0
        for epoch in range(epochs):
            indices = torch.randperm(batch_size, device=device)
            for start in range(0, batch_size, mini_batch_size):
                end = start + mini_batch_size
                mb_idx = indices[start:end]
                
                new_log_probs, values, entropy = ac.evaluate(b_obs[mb_idx], b_actions[mb_idx])
                
                ratio = torch.exp(new_log_probs - b_log_probs[mb_idx])
                surr1 = ratio * b_advs[mb_idx]
                surr2 = torch.clamp(ratio, 1.0 - clip_param, 1.0 + clip_param) * b_advs[mb_idx]
                actor_loss = -torch.min(surr1, surr2).mean()
                
                value_pred_clipped = b_values[mb_idx] + (values - b_values[mb_idx]).clamp(-clip_param, clip_param)
                value_losses = (values - b_returns[mb_idx]).pow(2)
                value_losses_clipped = (value_pred_clipped - b_returns[mb_idx]).pow(2)
                critic_loss = torch.max(value_losses, value_losses_clipped).mean()
                
                entropy_loss = -0.01 * entropy.mean()
                loss = actor_loss + 1.0 * critic_loss + entropy_loss
                
                optimizer.zero_grad()
                loss.backward()
                nn.utils.clip_grad_norm_(ac.parameters(), max_grad_norm)
                optimizer.step()
                
                with torch.no_grad():
                    kl = (b_log_probs[mb_idx] - new_log_probs).mean()
                    mean_kl += kl.item()
                    
        mean_kl /= (epochs * (batch_size // mini_batch_size))
        
        if mean_kl > desired_kl * 2.0:
            lr = max(1e-5, lr / 1.5)
            for param_group in optimizer.param_groups:
                param_group['lr'] = lr
        elif mean_kl < desired_kl / 2.0 and lr < 1e-3:
            lr = min(1e-3, lr * 1.5)
            for param_group in optimizer.param_groups:
                param_group['lr'] = lr
                
        t1 = time.time()
        fps = (num_envs * num_steps) / (t1 - t0)
        print(f"Iter: {it:03d} | Reward: {total_reward/num_steps:.3f} | FPS: {fps:.0f} | KL: {mean_kl:.4f} | LR: {lr:.2e}")
        
        # Log to TensorBoard
        writer.add_scalar("Perf/FPS", fps, it)
        writer.add_scalar("Perf/LR", lr, it)
        writer.add_scalar("Perf/KL", mean_kl, it)
        for k, v in epoch_metrics.items():
            writer.add_scalar(k, v / num_steps, it)
        
        if (it + 1) % 100 == 0:
            os.makedirs("checkpoints", exist_ok=True)
            save_path = os.path.join("checkpoints", f"student_policy_iter{it+1}.pt")
            torch.save(ac.state_dict(), save_path)
            print(f"Saved student policy to {save_path}")
