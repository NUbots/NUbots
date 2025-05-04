import distutils.util
import functools
import os
import re
import subprocess

import mujoco
from brax.training.agents.ppo import networks as ppo_networks
from brax.training.agents.ppo import train as ppo
from mujoco_playground import registry, wrapper
from mujoco_playground.config import locomotion_params

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
    print("Started training a joystick policy for NUgus")
    train_bipedal_joystick_policy()

def train_bipedal_joystick_policy():
    """
    Trains a joystick policy for a bipedal robot using MuJoCo Playground and Brax PPO.
    """
    # Choose the bipedal environment
    env_name = 'BerkeleyHumanoidJoystickFlatTerrain'
    env = registry.load(env_name)
    env_cfg = registry.get_default_config(env_name)
    ppo_params = locomotion_params.brax_ppo_config(env_name)

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
        print(f"Step: {num_steps}, Eval Reward: {metrics.get('eval/episode_reward', 'N/A')}")

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
    print("Training complete!")
    print("Final metrics:", metrics)
