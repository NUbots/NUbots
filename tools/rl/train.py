import distutils.util
import functools
import json
import os
import re
import subprocess
import sys
import time
from datetime import datetime

import jax
import mediapy as media
import mujoco
import numpy as np
from brax.training.agents.ppo import networks as ppo_networks
from brax.training.agents.ppo import train as ppo
from etils import epath
from flax.training import orbax_utils
from jax import numpy as jp
from ml_collections import config_dict
from mujoco_playground import registry, wrapper
from mujoco_playground._src.gait import draw_joystick_command
from mujoco_playground.config import locomotion_params
from orbax import checkpoint as ocp

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
# from common.export_onnx import export_onnx
from nugus.joystick import Joystick, nugus_env_config
from nugus.ppo_config import nugus_ppo_config

from utility.dockerise import run_on_docker

# Configure MuJoCo to use the EGL rendering backend (requires GPU)
os.environ['MUJOCO_GL'] = 'egl'

# Tell XLA to use Triton GEMM, this improves steps/sec by ~30% on some GPUs
xla_flags = os.environ.get('XLA_FLAGS', '')
xla_flags += ' --xla_gpu_triton_gemm_any=True'
os.environ['XLA_FLAGS'] = xla_flags

def register(command):
    command.description = "Train a RL joystick policy for NUgus"
    command.add_argument("--play_only", action="store_true", help="Only run inference with a trained model without training")
    command.add_argument("--load_checkpoint_path", type=str, help="Path to checkpoint to load for inference or continued training")

def run(**kwargs):
    """
    Trains a joystick policy for a bipedal robot using MuJoCo Playground and Brax PPO.
    """
    play_only = kwargs.get("play_only", False)
    load_checkpoint_path = kwargs.get("load_checkpoint_path", None)

    for device in jax.devices():
        print(f"Device: {device}, Type: {device.device_kind}, Platform: {device.platform}")

    if play_only:
        print("Running in play-only mode - no training will be performed")
        if not load_checkpoint_path:
            print("Error: In play_only mode, you must specify a checkpoint to load with --load_checkpoint_path")
            return

    print(f"{'Playing' if play_only else 'Training'} a joystick policy for NUgus on device: {jax.default_backend()}")

    if not load_checkpoint_path:
        # Setup experiment name and logging directories
        now = datetime.now()
        timestamp = now.strftime("%Y%m%d-%H%M%S")
        exp_name = f"NugusJoystick-{timestamp}"

        # Set up logging directory
        logdir = epath.Path("/home/nubots/build/recordings/rl").resolve() / exp_name
        logdir.mkdir(parents=True, exist_ok=True)
        print(f"Logs are being stored in: {logdir}")

        # Set up checkpoint directory
        ckpt_path = logdir / "checkpoints"
        ckpt_path.mkdir(parents=True, exist_ok=True)
        print(f"Checkpoint path: {ckpt_path}")
    else:
        ckpt_path = epath.Path(load_checkpoint_path)
        print(f"Loading checkpoint from: {ckpt_path}")

    train_bipedal_joystick_policy(ckpt_path, play_only=play_only, load_checkpoint_path=load_checkpoint_path)

def train_bipedal_joystick_policy(ckpt_path=None, play_only=False, load_checkpoint_path=None):
    """
    Trains a joystick policy for a bipedal robot using MuJoCo Playground and Brax PPO.

    Args:
        ckpt_path: Path to save checkpoints to.
        play_only: If True, only run inference without training.
        load_checkpoint_path: Path to load checkpoint from.
    """
    # Choose the bipedal environment
    env_name = 'NugusJoystick'
    env = Joystick()
    env_cfg = nugus_env_config()

    # Save environment configuration if checkpoint path is provided
    if ckpt_path is not None:
        with open(ckpt_path / "config.json", "w", encoding="utf-8") as fp:
            json.dump(env_cfg.to_dict(), fp, indent=4)

    # Get PPO config and set episode length from env_cfg
    ppo_params = nugus_ppo_config() # TODO: Figure out how to get the PPO config locally
    ppo_params.episode_length = env_cfg.episode_length

    # If in play_only mode, set num_timesteps to 0 to skip training
    if play_only:
        ppo_params.num_timesteps = 0

    # Get domain randomizer if available
    randomizer = registry.get_domain_randomizer(env_name)
    ppo_training_params = dict(ppo_params)
    network_factory = ppo_networks.make_ppo_networks
    if "network_factory" in ppo_params:
        del ppo_training_params["network_factory"]
        network_factory = functools.partial(
            ppo_networks.make_ppo_networks,
            **ppo_params.network_factory
        )

    # Define policy parameters function for saving checkpoints
    def policy_params_fn(current_step, make_policy, params):
        if ckpt_path is not None:
            orbax_checkpointer = ocp.PyTreeCheckpointer()
            save_args = orbax_utils.save_args_from_target(params)
            path = ckpt_path / f"{current_step}"
            orbax_checkpointer.save(path, params, force=True, save_args=save_args)
            print(f"Saved checkpoint at step {current_step} to {path}")
            # onnx_path = ckpt_path / f"{current_step}.onnx"
            # export_onnx(
            #     params,
            #     env.action_size,
            #     ppo_params,
            #     env.obs_size,  # may not work
            #     output_path=onnx_path
            # )
    # Use monotonic time for more accurate timing
    times = [time.monotonic()]

    def progress(num_steps, metrics):
        times.append(time.monotonic())
        print(f"Step: {num_steps}, Eval #{metrics.get('eval/episode', 'N/A')}, Eval Reward: {metrics.get('eval/episode_reward', 'N/A')}")

    # Handle checkpoint loading
    restore_checkpoint_path = None
    if load_checkpoint_path is not None:
        ckpt_path_obj = epath.Path(load_checkpoint_path)
        if ckpt_path_obj.is_dir():
            # Try to find latest checkpoint in directory
            latest_ckpts = list(ckpt_path_obj.glob("*"))
            latest_ckpts = [ckpt for ckpt in latest_ckpts if ckpt.is_dir()]
            if latest_ckpts:
                try:
                    # Sort by numeric value if possible
                    latest_ckpts.sort(key=lambda x: int(x.name))
                except ValueError:
                    # Otherwise sort by name
                    latest_ckpts.sort()
                restore_checkpoint_path = latest_ckpts[-1]
                print(f"Restoring from latest checkpoint: {restore_checkpoint_path}")
            else:
                print(f"No checkpoints found in {load_checkpoint_path}")
        else:
            restore_checkpoint_path = ckpt_path_obj
            print(f"Restoring from checkpoint: {restore_checkpoint_path}")

    # Set up the training function
    train_fn = functools.partial(
        ppo.train, **dict(ppo_training_params),
        network_factory=network_factory,
        randomization_fn=randomizer,
        progress_fn=progress,
        policy_params_fn=policy_params_fn,
        restore_checkpoint_path=restore_checkpoint_path
    )

    # Run training
    make_inference_fn, params, metrics = train_fn(
        environment=env,
        eval_env=registry.load(env_name, config=env_cfg),
        wrap_env_fn=wrapper.wrap_for_brax_training,
    )

    if len(times) > 1:
        print(f"Time to JIT:   {times[1] - times[0]:.2f} seconds")
        print(f"Time to train: {times[-1] - times[1]:.2f} seconds")

    if play_only:
        print("Skipped training, loaded model from checkpoint")
    else:
        print("Training complete!")
        print("Final metrics:", metrics)

    # Save final checkpoint
    if ckpt_path is not None and not play_only:
        orbax_checkpointer = ocp.PyTreeCheckpointer()
        save_args = orbax_utils.save_args_from_target(params)
        path = ckpt_path / "final"
        orbax_checkpointer.save(path, params, force=True, save_args=save_args)
        print(f"Saved final checkpoint to {path}")

    # Rollout and render
    eval_env = registry.load(env_name, config=env_cfg)
    jit_reset = jax.jit(eval_env.reset)
    jit_step = jax.jit(eval_env.step)
    jit_inference_fn = jax.jit(make_inference_fn(params, deterministic=True))

    rng = jax.random.PRNGKey(1)

    rollout = []
    modify_scene_fns = []

    x_vel = 0.5
    y_vel = 0.0
    yaw_vel = 0.0
    command = jp.array([x_vel, y_vel, yaw_vel])

    phase_dt = 2 * jp.pi * eval_env.dt * 1.5
    phase = jp.array([0, jp.pi])

    for j in range(1):
        print(f"episode {j}")
        state = jit_reset(rng)
        state.info["phase_dt"] = phase_dt
        state.info["phase"] = phase
        for i in range(env_cfg.episode_length):
            act_rng, rng = jax.random.split(rng)
            ctrl, _ = jit_inference_fn(state.obs, act_rng)
            state = jit_step(state, ctrl)
            if state.done:
                break
            state.info["command"] = command
            rollout.append(state)

            xyz = np.array(state.data.xpos[eval_env.mj_model.body("torso").id])
            xyz += np.array([0, 0.0, 0])
            x_axis = state.data.xmat[eval_env._torso_body_id, 0]
            yaw = -np.arctan2(x_axis[1], x_axis[0])
            modify_scene_fns.append(
                functools.partial(
                    draw_joystick_command,
                    cmd=state.info["command"],
                    xyz=xyz,
                    theta=yaw,
                    scl=np.linalg.norm(state.info["command"]),
                )
            )

    render_every = 1
    fps = 1.0 / eval_env.dt / render_every
    print(f"fps: {fps}")
    traj = rollout[::render_every]
    mod_fns = modify_scene_fns[::render_every]

    scene_option = mujoco.MjvOption()
    scene_option.geomgroup[2] = True
    scene_option.geomgroup[3] = False
    scene_option.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = True
    scene_option.flags[mujoco.mjtVisFlag.mjVIS_TRANSPARENT] = False
    scene_option.flags[mujoco.mjtVisFlag.mjVIS_PERTFORCE] = False

    frames = eval_env.render(
        traj,
        camera="track",
        scene_option=scene_option,
        width=640*2,
        height=480,
        modify_scene_fns=mod_fns,
    )

    # Save video to file rather than displaying it
    video_path = ckpt_path.parent / "rollout.mp4" if ckpt_path else "rollout.mp4"
    media.write_video(video_path, frames, fps=fps)
    print(f"Rollout video saved to {video_path}")

    # Export final policy to onnx
