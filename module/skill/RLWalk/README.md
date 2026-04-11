# RLWalk

## Description

RLWalk is a walking skill that runs a trained reinforcement-learning policy exported as an ONNX model and executed via OpenVINO.
On each policy update, it builds an observation vector from IMU and joint state, runs inference to predict joint offsets, applies safety clipping, and emits servo commands.

## Usage

Include this module in your role to provide walking behaviour for `message::skill::Walk`.

To run keyboardwalk using the host-side MuJoCo bridge/viewer, set `module/platform/Mujoco/data/config/Mujoco.yaml`:

`./b run mujoco/rl_keyboardwalk --environment DISPLAY=$DISPLAY --volume /tmp/.X11-unix:/tmp/.X11-unix:rw --gpus all`

```yaml
mode: "remote"
remote_host: "172.17.0.1"
remote_port: "18080"
```

Then run:

`./b run mujoco/rl_keyboardwalk`

Start the host bridge with the interactive MuJoCo viewer enabled (default):

`python run_bridge.py --world /absolute/path/to/nugus/scene.xml --host 0.0.0.0 --port 18080`

If you need headless mode for debugging or CI, use:

`python run_bridge.py --no-viewer --world /absolute/path/to/nugus/scene.xml --host 0.0.0.0 --port 18080`

For local in-container rendering (legacy path), keep `mode: "local"` and run with X11 forwarding:

`./b run mujoco/rl_keyboardwalk --environment DISPLAY=$DISPLAY --volume /tmp/.X11-unix:/tmp/.X11-unix:rw --gpus all`

Model and actuator parameters are loaded from `module/skill/RLWalk/data/config/RLWalk.yaml` (e.g. `model.path`, `model.device`, `servos.gain`, `servos.torque`, `default_pose`).

## Consumes

- `message::skill::Walk` A walk task containing `velocity_target` (desired linear/rotational velocity).
- `message::input::Sensors` Sensor inputs used to build the policy observation (gyro, gravity-in-body-frame, and servo present position/velocity).
- `message::behaviour::state::Stability` Used to gate policy execution; RL inference runs only when stability is at least `Stability::DYNAMIC`.

## Emits

- `message::actuation::Body` Servo command body constructed from policy output (policy-predicted joint offsets added to `default_pose`, then converted to the NUbots joint ordering).
- `message::behaviour::state::WalkState` `STOPPED` when the walk task is started/stopped and `WALKING` while actively walking.
- `message::behaviour::state::Stability` An initial `Stability::UNKNOWN` at startup (so downstream walk behaviours can bootstrap).

## Dependencies

- OpenVINO (ONNX model inference)
- Eigen
- TBB
- Director
