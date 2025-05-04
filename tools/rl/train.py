import distutils.util
import functools
import os
import re
import subprocess
import sys
from datetime import datetime

import jax
import mujoco
import numpy as np
from brax.training.agents.ppo import networks as ppo_networks
from brax.training.agents.ppo import train as ppo
from jax import numpy as jp
from ml_collections import config_dict
from mujoco_playground import registry, wrapper
from mujoco_playground._src.gait import draw_joystick_command
from mujoco_playground.config import locomotion_params

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from nugus.joystick import Joystick, default_config

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
    train_bipedal_joystick_policy()

def train_bipedal_joystick_policy():
    """
    Trains a joystick policy for a bipedal robot using MuJoCo Playground and Brax PPO.
    """
    # Choose the bipedal environment
    env_name = 'NugusJoystick'
    env = Joystick()
    env_cfg = default_config()
    ppo_params = config_dict.create(
        num_timesteps=100_000_000,
        num_evals=1,
        reward_scaling=1.0,
        episode_length=env_cfg.episode_length,
        normalize_observations=True,
        action_repeat=1,
        unroll_length=20,
        num_minibatches=32,
        num_updates_per_batch=4,
        discounting=0.97,
        learning_rate=3e-4,
        entropy_cost=1e-2,
        num_envs=8192,
        batch_size=256,
        max_grad_norm=1.0,
        network_factory=config_dict.create(
            policy_hidden_layer_sizes=(128, 128, 128, 128),
            value_hidden_layer_sizes=(256, 256, 256, 256, 256),
            policy_obs_key="state",
            value_obs_key="state",
        ),
    )

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

    def progress(num_steps, metrics):
        times.append(datetime.now())
        print(f"Step: {num_steps}, Eval Reward: {metrics.get('eval/episode_reward', 'N/A')}")

    times = [datetime.now()]
    # Set up the training function
    train_fn = functools.partial(
        ppo.train, **dict(ppo_training_params),
        network_factory=network_factory,
        randomization_fn=randomizer,
        progress_fn=progress
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
    media.show_video(frames, fps=fps, loop=False)

    # Export policy to onnx
