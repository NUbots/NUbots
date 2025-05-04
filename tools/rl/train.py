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
import onnxruntime as rt
import tensorflow as tf
import tf2onnx
from brax.training.acme import running_statistics
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
from tensorflow.keras import layers

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
    export_policy_to_onnx(params, env, make_inference_fn, ckpt_path)

def export_policy_to_onnx(params, env, make_inference_fn, ckpt_path=None):
    """
    Export the trained policy model to ONNX format.

    Args:
        params: The trained model parameters
        env: The environment instance
        make_inference_fn: Function to create the inference function
        ckpt_path: Path to save the ONNX model
    """
    print("Exporting policy to ONNX format...")

    # Define output path
    output_path = ckpt_path.parent / "nugus_policy.onnx" if ckpt_path else "nugus_policy.onnx"

    # Get observation and action sizes
    obs_size = env.observation_size
    act_size = env.action_size
    print(f"Observation size: {obs_size}, Action size: {act_size}")

    # Extract normalization parameters
    normalizer_params = params[0] if isinstance(params, tuple) else {}
    policy_params = params[1] if isinstance(params, tuple) else params

    # Create TensorFlow model equivalent
    mean = normalizer_params.mean["state"] if hasattr(normalizer_params, 'mean') else None
    std = normalizer_params.std["state"] if hasattr(normalizer_params, 'std') else None

    # Create TensorFlow MLP model
    tf_policy_network = create_tf_policy_network(
        param_size=act_size * 2,
        mean_std=(tf.convert_to_tensor(mean), tf.convert_to_tensor(std)) if mean is not None and std is not None else None,
        hidden_layer_sizes=ppo_params.network_factory.policy_hidden_layer_sizes if hasattr(ppo_params, 'network_factory') else [128, 128, 128, 128],
        activation=tf.nn.swish
    )

    # Transfer weights from JAX model to TensorFlow model
    transfer_weights(policy_params['params'], tf_policy_network)

    # Define the TensorFlow input signature
    spec = [tf.TensorSpec(shape=(1, obs_size["state"][0]), dtype=tf.float32, name="obs")]

    # Set output names
    tf_policy_network.output_names = ['continuous_actions']

    # Convert to ONNX
    model_proto, _ = tf2onnx.convert.from_keras(tf_policy_network, input_signature=spec, opset=11, output_path=str(output_path))

    # Verify the ONNX model
    providers = ['CPUExecutionProvider']
    try:
        m = rt.InferenceSession(str(output_path), providers=providers)
        onnx_input = {'obs': np.ones((1, obs_size["state"][0]), dtype=np.float32)}
        onnx_pred = m.run(['continuous_actions'], onnx_input)[0]
        print(f"ONNX model verification: Output shape {onnx_pred.shape}")
        print(f"ONNX model saved to {output_path}")
    except Exception as e:
        print(f"Error verifying ONNX model: {e}")

def create_tf_policy_network(param_size, mean_std=None, hidden_layer_sizes=[128, 128, 128, 128], activation=tf.nn.relu):
    """
    Create a TensorFlow MLP policy network.

    Args:
        param_size: Size of the output parameters
        mean_std: Tuple of (mean, std) for observation normalization
        hidden_layer_sizes: List of hidden layer sizes
        activation: Activation function

    Returns:
        TensorFlow MLP model
    """
    class MLP(tf.keras.Model):
        def __init__(
            self,
            layer_sizes,
            activation=tf.nn.relu,
            kernel_init="lecun_uniform",
            activate_final=False,
            bias=True,
            layer_norm=False,
            mean_std=None,
        ):
            super().__init__()

            self.layer_sizes = layer_sizes
            self.activation = activation
            self.kernel_init = kernel_init
            self.activate_final = activate_final
            self.bias = bias
            self.layer_norm = layer_norm

            if mean_std is not None:
                self.mean = tf.Variable(mean_std[0], trainable=False, dtype=tf.float32)
                self.std = tf.Variable(mean_std[1], trainable=False, dtype=tf.float32)
            else:
                self.mean = None
                self.std = None

            self.mlp_block = tf.keras.Sequential(name="MLP_0")
            for i, size in enumerate(self.layer_sizes):
                dense_layer = layers.Dense(
                    size,
                    activation=self.activation if i < len(self.layer_sizes) - 1 or activate_final else None,
                    kernel_initializer=self.kernel_init,
                    name=f"hidden_{i}",
                    use_bias=self.bias,
                )
                self.mlp_block.add(dense_layer)
                if self.layer_norm:
                    self.mlp_block.add(layers.LayerNormalization(name=f"layer_norm_{i}"))

            self.submodules = [self.mlp_block]

        def call(self, inputs):
            if isinstance(inputs, list):
                inputs = inputs[0]
            if self.mean is not None and self.std is not None:
                inputs = (inputs - self.mean) / self.std
            logits = self.mlp_block(inputs)
            loc, _ = tf.split(logits, 2, axis=-1)
            return tf.tanh(loc)

    policy_network = MLP(
        layer_sizes=list(hidden_layer_sizes) + [param_size],
        activation=activation,
        kernel_init="lecun_uniform",
        layer_norm=False,
        mean_std=mean_std,
    )
    return policy_network

def transfer_weights(jax_params, tf_model):
    """
    Transfer weights from a JAX parameter dictionary to the TensorFlow model.

    Args:
        jax_params: JAX parameter dictionary
        tf_model: TensorFlow model
    """
    for layer_name, layer_params in jax_params.items():
        try:
            tf_layer = tf_model.get_layer("MLP_0").get_layer(name=layer_name)
        except ValueError:
            print(f"Layer {layer_name} not found in TensorFlow model.")
            continue
        if isinstance(tf_layer, tf.keras.layers.Dense):
            kernel = np.array(layer_params['kernel'])
            bias = np.array(layer_params['bias'])
            print(f"Transferring Dense layer {layer_name}, kernel shape {kernel.shape}, bias shape {bias.shape}")
            tf_layer.set_weights([kernel, bias])
        else:
            print(f"Unhandled layer type in {layer_name}: {type(tf_layer)}")

    print("Weights transferred successfully.")
