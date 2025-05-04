import distutils.util
import functools
import json
import os
import re
import subprocess
import sys
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
from nugus.joystick import Joystick, default_config
from nugus.ppo_config import get_default_ppo_config

from utility.dockerise import run_on_docker

# Configure MuJoCo to use the EGL rendering backend (requires GPU)
os.environ['MUJOCO_GL'] = 'egl'

# Tell XLA to use Triton GEMM, this improves steps/sec by ~30% on some GPUs
xla_flags = os.environ.get('XLA_FLAGS', '')
xla_flags += ' --xla_gpu_triton_gemm_any=True'
os.environ['XLA_FLAGS'] = xla_flags


@run_on_docker
def register(command):
    command.description = "Train a RL joystick policy for NUgus"

@run_on_docker
def run(**kwargs):
    """
    Trains a joystick policy for a bipedal robot using MuJoCo Playground and Brax PPO.
    """
    for device in jax.devices():
        print(f"Device: {device}, Type: {device.device_kind}, Platform: {device.platform}")
    print("Started training a joystick policy for NUgus on device:", jax.default_backend())

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

    train_bipedal_joystick_policy(ckpt_path)

def train_bipedal_joystick_policy(ckpt_path=None):
    """
    Trains a joystick policy for a bipedal robot using MuJoCo Playground and Brax PPO.

    Args:
        ckpt_path: Path to save checkpoints to.
    """
    # Choose the bipedal environment
    env_name = 'NugusJoystick'
    env = Joystick()
    env_cfg = default_config()

    # Save environment configuration if checkpoint path is provided
    if ckpt_path is not None:
        with open(ckpt_path / "config.json", "w", encoding="utf-8") as fp:
            json.dump(env_cfg.to_dict(), fp, indent=4)

    # Get PPO config and set episode length from env_cfg
    ppo_params = get_default_ppo_config()
    ppo_params.episode_length = env_cfg.episode_length

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

    def progress(num_steps, metrics):
        times.append(datetime.now())
        print(f"Step: {num_steps}, Eval Reward: {metrics.get('eval/episode_reward', 'N/A')}")

    times = [datetime.now()]
    # Set up the training function
    train_fn = functools.partial(
        ppo.train, **dict(ppo_training_params),
        network_factory=network_factory,
        randomization_fn=randomizer,
        progress_fn=progress,
        policy_params_fn=policy_params_fn
    )

    # Run training
    make_inference_fn, params, metrics = train_fn(
        environment=env,
        eval_env=registry.load(env_name, config=env_cfg),
        wrap_env_fn=wrapper.wrap_for_brax_training,
    )
    print("Time to jit", times[1] - times[0])
    print("Time to train", times[-1] - times[1])
    print("Training complete!")
    print("Final metrics:", metrics)

    # Save final checkpoint
    if ckpt_path is not None:
        orbax_checkpointer = ocp.PyTreeCheckpointer()
        save_args = orbax_utils.save_args_from_target(params)
        path = ckpt_path / "final"
        orbax_checkpointer.save(path, params, force=True, save_args=save_args)
        print(f"Saved final checkpoint to {path}")

    # Rollout and render
    env = registry.load(env_name)
    eval_env = registry.load(env_name)
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
