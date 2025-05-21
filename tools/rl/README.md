# RL Training

This directory contains a script to train a joystick policy for a bipedal robot using MuJoCo Playground and Brax PPO.

## How to Train

To start training run:
`./b rl train --gpus all`

This command ensures that GPUs are available inside the Docker container for faster training.

## How to create video of a single rollout
`./b rl train --gpus all --play_only --load_checkpoint_path=/home/nubots/build/recordings/rl/NugusJoystick-20250504-083351/checkpoints --volume /tmp/.X11-unix:/tmp/.X11-unix --env DISPLAY=$DISPLAY`

## Convert model to onnx

`./b rl export_model --checkpoint_path /home/nubots/NUbots/recordings/rl/NugusJoystick-20250517-153803/checkpoints`

## Control robot in python mujoco sim with onnx runtime

`./b rl sim --onnx_model_path path_to_model.onnx`


Updating playground after making changes on branch
`uv lock --upgrade-package playground`
