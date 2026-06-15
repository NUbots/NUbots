#!/usr/bin/env python3

import os
import time
import argparse
import numpy as np
import torch
import mujoco
import mujoco.viewer
from glfw import KEY_UP, KEY_DOWN, KEY_LEFT, KEY_RIGHT, KEY_Q, KEY_W, KEY_A, KEY_S, KEY_D

from .model import WalkPolicy, ActorCritic

class WalkEngineState:
    def __init__(self, step_period=0.32):
        self.step_period = step_period
        self.phase_time = 0.0
        self.phase_indicator = 1.0 # LEFT
        self.engine_state = 0 # STOPPED

    def update(self, dt, target_velocity):
        is_velocity_zero = (np.linalg.norm(target_velocity) < 1e-4)

        if is_velocity_zero and self.phase_time < self.step_period:
            self.engine_state = 3 # STOPPING
        elif self.phase_time >= self.step_period and (is_velocity_zero or self.engine_state == 3):
            self.engine_state = 0 # STOPPED
        elif not is_velocity_zero and self.engine_state == 0:
            self.engine_state = 1 # STARTING
            self.phase_time = 0.0

        if self.engine_state == 0:
            self.phase_time = self.step_period
        else:
            self.phase_time += dt
            if self.phase_time >= self.step_period and self.engine_state != 3:
                self.phase_time -= self.step_period
                self.phase_indicator = -self.phase_indicator
                if self.engine_state == 1:
                    self.engine_state = 2 # WALKING

        return self.get_obs()

    def get_obs(self):
        phase_ratio = self.phase_time / self.step_period if self.step_period > 0 else 0.0
        phase_sin = np.sin(2.0 * np.pi * phase_ratio)
        phase_cos = np.cos(2.0 * np.pi * phase_ratio)

        state_onehot = np.zeros(4, dtype=np.float32)
        state_onehot[self.engine_state] = 1.0

        return phase_sin, phase_cos, self.phase_indicator, state_onehot

def register(command):
    command.description = "Test the distilled policy in an interactive MuJoCo viewer"

    command.add_argument(
        "--checkpoint", "-c",
        type=str,
        default="checkpoints/best_model.pth",
        help="Path to the trained model checkpoint (.pth)"
    )
    command.add_argument(
        "--stats", "-s",
        type=str,
        default="checkpoints/dataset_stats.npz",
        help="Path to the dataset statistics (.npz)"
    )
    command.add_argument(
        "--rl-checkpoint", "-r",
        type=str,
        default=None,
        help="Path to the trained student RL policy (.pt)"
    )

def run(checkpoint, stats, rl_checkpoint, **kwargs):
    # 1. Load the model and stats
    print("Loading model and stats...")
    device = torch.device("cpu")
    stats_data = np.load(stats)

    obs_mean = torch.from_numpy(stats_data["obs_mean"]).float().to(device)
    obs_std = torch.from_numpy(stats_data["obs_std"]).float().to(device)
    target_mean = torch.from_numpy(stats_data["target_mean"]).float().to(device)
    target_std = torch.from_numpy(stats_data["target_std"]).float().to(device)

    obs_std[obs_std < 1e-8] = 1.0
    target_std[target_std < 1e-8] = 1.0

    model = WalkPolicy(obs_dim=len(obs_mean), target_dim=12).to(device)
    checkpoint_data = torch.load(checkpoint, map_location=device)
    model.load_state_dict(checkpoint_data.get("model_state_dict", checkpoint_data))
    model.eval()

    rl_model = None
    if rl_checkpoint and os.path.exists(rl_checkpoint):
        print(f"Loading RL student policy from {rl_checkpoint}...")
        # commands(3) + proj_g(3) + lin_v(3) + ang_v(3) + qpos_err(20) + qvel(20) + last_actions(20) = 72
        rl_model = ActorCritic(num_obs=72, num_actions=12).to(device)
        rl_model.load_state_dict(torch.load(rl_checkpoint, map_location=device))
        rl_model.eval()

    # 2. Load MuJoCo model
    mj_model = mujoco.MjModel.from_xml_path("tools/walk_distillation/mujoco/scene.xml")

    mj_data = mujoco.MjData(mj_model)
    
    key_id = mujoco.mj_name2id(mj_model, mujoco.mjtObj.mjOBJ_KEY, "stand_bent_knees")
    if key_id != -1:
        mujoco.mj_resetDataKeyframe(mj_model, mj_data, key_id)
    else:
        print("Warning: Keyframe 'stand_bent_knees' not found!")
        
    mujoco.mj_forward(mj_model, mj_data)

    # 3. State tracking
    dt = 0.01  # 100Hz for policy
    walk_state = WalkEngineState(step_period=0.32)

    # We need 3 history frames of 12 joints
    # Wait, the robot's default pose is not 0 for all joints, it is the standing pose.
    # Let's get the standing pose from the first few frames of evaluation, or just use 0.
    # Actually, we can get it from the dataset stats! target_mean is the average pose.
    history = [stats_data["target_mean"].copy() for _ in range(3)]
    
    # Store last RL actions (actually 20 elements because env.py has self.NU = 20 for NUgus)
    last_actions = mj_data.ctrl.copy()

    # Target velocity controlled by keyboard
    target_velocity = np.zeros(3, dtype=np.float32)
    current_velocity = np.zeros(3, dtype=np.float32)
    
    default_qpos = None
    key_id = mujoco.mj_name2id(mj_model, mujoco.mjtObj.mjOBJ_KEY, "stand_bent_knees")
    if key_id != -1:
        default_qpos = mj_model.key_qpos[key_id].copy()
    else:
        default_qpos = np.zeros(mj_model.nq)

    def get_rl_obs(target_vel, mj_model, mj_data, default_qpos, last_actions):
        # Base quaternion q = [w, x, y, z]
        qw, qx, qy, qz = mj_data.qpos[3:7]
        
        # quat_rotate_inverse implementation in numpy
        # q_vec_inv = [-qx, -qy, -qz]
        def quat_rotate_inverse(q, v):
            q_vec_inv = np.array([-q[1], -q[2], -q[3]])
            uv = np.cross(q_vec_inv, v)
            uuv = np.cross(q_vec_inv, uv)
            return v + 2.0 * (q[0] * uv + uuv)
            
        global_gravity = np.array([0.0, 0.0, -1.0])
        proj_gravity = quat_rotate_inverse(mj_data.qpos[3:7], global_gravity)
        
        base_lin_vel = quat_rotate_inverse(mj_data.qpos[3:7], mj_data.qvel[0:3])
        base_ang_vel = quat_rotate_inverse(mj_data.qpos[3:7], mj_data.qvel[3:6])
        
        obs = np.concatenate([
            target_vel,
            proj_gravity,
            base_lin_vel,
            base_ang_vel,
            mj_data.qpos[7:] - default_qpos[7:],
            mj_data.qvel[6:],
            last_actions
        ])
        return torch.tensor(obs, dtype=torch.float32, device=device).unsqueeze(0)

    def keyboard_callback(keycode):
        nonlocal target_velocity
        # W/S for vx, A/D for vy, Q/E for vtheta
        if keycode == KEY_W: target_velocity[0] = 0.15
        elif keycode == KEY_S: target_velocity[0] = -0.15
        elif keycode == KEY_A: target_velocity[1] = 0.1
        elif keycode == KEY_D: target_velocity[1] = -0.1
        elif keycode == KEY_Q: target_velocity[2] = 0.3
        elif keycode == 69: target_velocity[2] = -0.3 # KEY_E is 69
        # Reset velocity on key release? GLFW callbacks in mujoco viewer are just press events
        # We will reset to 0 if SPACE is pressed
        elif keycode == 32: # SPACE
            target_velocity *= 0.0

    print("Starting viewer...")
    print("Controls: W/S (forward/back), A/D (strafe), Q/E (turn), SPACE (stop)")

    with mujoco.viewer.launch_passive(mj_model, mj_data, key_callback=keyboard_callback) as viewer:
        # Initial settle
        for _ in range(100):
            mujoco.mj_step(mj_model, mj_data)

        last_update = time.time()

        while viewer.is_running():
            step_start = time.time()

            # Smooth velocity (simulating Walk.cpp)
            dv = np.array([1.5, 1.5, 3.0]) * dt
            current_velocity = np.clip(current_velocity + np.clip(target_velocity - current_velocity, -dv, dv), -1, 1)

            # Update state machine
            phase_sin, phase_cos, phase_ind, state_onehot = walk_state.update(dt, current_velocity)

            # Construct observation
            obs = np.concatenate([
                current_velocity,
                [phase_sin, phase_cos, phase_ind],
                state_onehot,
                history[0], history[1], history[2]
            ]).astype(np.float32)

            obs_tensor = torch.from_numpy(obs).unsqueeze(0).to(device)
            norm_obs = (obs_tensor - obs_mean) / obs_std

            with torch.no_grad():
                norm_target = model(norm_obs)

            pred_target = (norm_target * target_std) + target_mean
            teacher_joints = pred_target.squeeze(0).cpu().numpy()
            
            final_joints = teacher_joints
            
            if rl_model is not None:
                rl_obs = get_rl_obs(current_velocity, mj_model, mj_data, default_qpos, last_actions)
                with torch.no_grad():
                    actions, _ = rl_model(rl_obs)
                residuals = actions.squeeze(0).cpu().numpy()
                final_joints = teacher_joints + 0.2 * np.tanh(residuals)

            # Update history (shift)
            history[2] = history[1]
            history[1] = history[0]
            history[0] = final_joints.copy()

            # Set MJ control (actuators match the 12 leg joints, followed by 8 arm/head joints)
            mj_data.ctrl[:12] = final_joints
            last_actions[:12] = final_joints

            # Step physics
            # Physics is 0.002s, policy is 0.01s, so step physics 5 times
            for _ in range(5):
                mujoco.mj_step(mj_model, mj_data)

            viewer.sync()

            # Sleep to match real time
            time_until_next_step = dt - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)
