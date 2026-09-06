# RLWalk

## Description

RLWalk is a walking skill that runs a trained reinforcement-learning policy exported as an ONNX model and executed via OpenVINO.
On each policy update, it builds an observation vector from IMU and joint state, runs inference to predict joint offsets, applies safety clipping, and emits servo commands.

## Usage

Include `skill::RLWalk` in your role to provide walking behaviour.

## Consumes

- `message::skill::Walk` A walk task containing a vector of desired linear/rotational velocity.
- `message::input::Sensors` Sensor inputs used to build the policy observation (gyro, gravity-in-body-frame, and servo present position/velocity).
- `message::behaviour::state::Stability` Used to gate policy execution; RL inference runs only when stability is at least `Stability::DYNAMIC`.

## Emits

- `message::actuation::Limbs`: Sends servo commands constructed from policy output to non-head servos.
- `message::behaviour::state::WalkState` `STOPPED` when the walk task is started/stopped and `WALKING` while actively walking.
- `message::behaviour::state::Stability` An initial `Stability::UNKNOWN` at startup (so downstream walk behaviours can bootstrap).

## Dependencies

- OpenVINO (ONNX model inference for intel hardware)
- Eigen
- TBB
- Director
