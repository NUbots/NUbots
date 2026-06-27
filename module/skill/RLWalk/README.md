# RLWalk

## Description

RLWalk is a walking skill that runs a trained reinforcement-learning policy exported as an ONNX model and executed via OpenVINO.
On each policy update, it builds an observation vector from IMU and joint state, runs inference to predict joint offsets, applies safety clipping, and emits servo commands.

## Usage

Include this module in your role to provide walking behaviour for `message::skill::Walk`.

To run keyboardwalk in using the mujoco viewer (deprecated):

`./b run mujoco/rl_keyboardwalk --environment DISPLAY=$DISPLAY --volume /tmp/.X11-unix:/tmp/.X11-unix:rw --gpus all`

`DISPLAY` + the `/tmp/.X11-unix` bind-mount allow the Mujoco GUI to display on the host. `--gpus all` is also passed through for GPU-accelerated execution where supported by your OpenVINO/container setup.

## Consumes

- `message::skill::Walk` A walk task containing a vector of desired linear/rotational velocity.
- `message::input::Sensors` Sensor inputs used to build the policy observation (gyro, gravity-in-body-frame, and servo present position/velocity).
- `message::behaviour::state::Stability` Used to gate policy execution; RL inference runs only when stability is at least `Stability::DYNAMIC`.

## Emits

- `message::actuation::Body` Servo command body constructed from policy output (policy-predicted joint offsets added to `default_pose`, then converted to the NUbots joint ordering).
- `message::behaviour::state::WalkState` `STOPPED` when the walk task is started/stopped and `WALKING` while actively walking.
- `message::behaviour::state::Stability` An initial `Stability::UNKNOWN` at startup (so downstream walk behaviours can bootstrap).

## Dependencies

- OpenVINO (ONNX model inference for intel hardware)
- Eigen
- TBB
- Director
