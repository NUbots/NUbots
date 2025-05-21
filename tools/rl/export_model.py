import os
import sys
import argparse
from pathlib import Path

import jax
import tensorflow as tf
import numpy as np
from etils import epath
from orbax import checkpoint as ocp
from flax.training import orbax_utils

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from nugus.joystick import Joystick, nugus_env_config
from nugus.ppo_config import nugus_ppo_config
from common.export_onnx import export_onnx

def register(command):
    command.description = "Export a trained RL model to ONNX format"
    command.add_argument("--checkpoint_path", type=str, required=True, help="Path to the checkpoint directory")
    command.add_argument("--output_path", type=str, default="model.onnx", help="Path to save the ONNX model")


def run(**kwargs):
    """
    Exports a trained RL model to ONNX format.
    """
    checkpoint_path = kwargs.get("checkpoint_path")
    output_path = kwargs.get("output_path", "model.onnx")

    print(f"Exporting model from checkpoint: {checkpoint_path}")
    print(f"Output path: {output_path}")

    # Setup environment and config
    env = Joystick()
    env_cfg = nugus_env_config()
    ppo_params = nugus_ppo_config()

    # Load checkpoint
    ckpt_path = epath.Path(checkpoint_path)
    if not ckpt_path.exists():
        print(f"Error: Checkpoint path {ckpt_path} does not exist")
        return

    # Find latest checkpoint if directory
    if ckpt_path.is_dir():
        latest_ckpts = list(ckpt_path.glob("*"))
        latest_ckpts = [ckpt for ckpt in latest_ckpts if ckpt.is_dir()]
        if latest_ckpts:
            try:
                latest_ckpts.sort(key=lambda x: int(x.name))
            except ValueError:
                latest_ckpts.sort()
            ckpt_path = latest_ckpts[-1]
            print(f"Using latest checkpoint: {ckpt_path}")
        else:
            print(f"No checkpoints found in {checkpoint_path}")
            return

    # Load checkpoint
    orbax_checkpointer = ocp.PyTreeCheckpointer()
    params = orbax_checkpointer.restore(ckpt_path)

    # Debug: Print params structure
    print("\nParams structure:")
    if isinstance(params, list) and len(params) >= 2:
        print("Params is a list with", len(params), "elements")
        print("First element keys:", list(params[0].keys()))
        print("Second element keys:", list(params[1].keys()))
    else:
        print("Params type:", type(params))
        print("Params structure:", params)

    # Export to ONNX
    print(f"\nExporting model to {output_path}")
    try:
        # Create the expected structure for export_onnx
        restructured_params = [
            type('obj', (object,), {
                'mean': {'state': params[0]['mean']},
                'std': {'state': params[0]['std']}
            }),
            type('obj', (object,), {
                'policy': {'params': params[1]['params']}
            })
        ]

        export_onnx(
            restructured_params,
            env.action_size,
            ppo_params,
            env.observation_size,
            output_path=output_path
        )
        print("Export complete!")
    except Exception as e:
        print(f"Error during export: {str(e)}")
        print("\nPlease try installing the following specific versions:")
        print("pip install numpy==1.23.5 tensorflow==2.12.0 tf2onnx==1.13.0")
        sys.exit(1)

if __name__ == "__main__":
    main()
