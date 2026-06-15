#!/usr/bin/env python3

import os
import time
import argparse
import numpy as np
import torch
import mujoco
import mujoco.viewer
from glfw import KEY_UP, KEY_DOWN, KEY_LEFT, KEY_RIGHT, KEY_Q, KEY_W, KEY_A, KEY_S, KEY_D

from .model import WalkPolicy

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

def run(checkpoint, stats, **kwargs):
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

    # Target velocity controlled by keyboard
    target_velocity = np.zeros(3, dtype=np.float32)
    current_velocity = np.zeros(3, dtype=np.float32)

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
            target_joints = pred_target.squeeze(0).cpu().numpy()

            # Update history (shift)
            history[2] = history[1]
            history[1] = history[0]
            history[0] = target_joints

            # Set MJ control (actuators match the 12 leg joints, followed by 8 arm/head joints)
            mj_data.ctrl[:12] = target_joints

            # Step physics
            # Physics is 0.002s, policy is 0.01s, so step physics 5 times
            for _ in range(5):
                mujoco.mj_step(mj_model, mj_data)

            viewer.sync()

            # Sleep to match real time
            time_until_next_step = dt - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)
