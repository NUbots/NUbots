# RL Training

This directory contains a script to train a joystick policy for a bipedal robot using MuJoCo Playground and Brax PPO.

## How to Run

To start training run:
`./b rl train --gpus all`

This command ensures that GPUs are available inside the Docker container for faster training.


`./b rl train --gpus all --play_only --load_checkpoint_path=/home/nubots/build/recordings/rl/NugusJoystick-20250504-083351/checkpoints --volume /tmp/.X11-unix:/tmp/.X11-unix --env DISPLAY=$DISPLAY`

Updating playground after making changes on branch
`uv lock --upgrade-package playground`
