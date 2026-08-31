# ProfileWalk

## Description

Starts the Director graph for the ProfileWalk scenario.

ProfileWalk walks the robot through a fixed profile of walk commands instead of taking them from a
human, so that the same set of velocity commands can be replayed for walk tuning, system
identification and repeatable data collection.

The profile is a list of **stages**, and each stage is a list of **segments**. A segment is a single
walk velocity target `[vx, vy, wz]` that is linearly ramped up to over `ramp_time`, held for
`hold_time`, and then ramped back down to a stop. Segments and stages are separated by a `rest_time`
of standing still. Everything, including the profile itself, is set in `ProfileWalk.yaml`, and saving
the file restarts the profile from the beginning.

The default profile is:

| Stage | Segments                       | vx     | vy     | wz     |
| ----- | ------------------------------ | ------ | ------ | ------ |
| 1     | Sagittal forwards, backwards   | ± 0.35 | 0      | 0      |
| 2     | Lateral left, right            | 0      | ± 0.20 | 0      |
| 3     | Turn left, right               | 0      | 0      | ± 0.60 |
| 4     | Forwards + lateral left, right | 0.25   | ± 0.15 | 0      |
| 5     | Forwards + turn left, right    | 0.25   | 0      | ± 0.40 |
| 6     | Left + turn left, right        | 0      | 0.15   | ± 0.40 |

### Falling

The robot's `Stability` state is watched for the whole profile. When the robot starts falling the
profile is paused and a zero walk command is held, which lets `FallRecovery` take over and get the
robot back up. Once the robot is standing again the rest of the stage it fell in is abandoned and the
profile picks up from the start of the next stage, so a stage that puts the robot on the ground is
never reattempted. The fall and get up take the place of the usual rest between stages; set
`recovery_time` above zero if the robot needs to settle before walking again.

## Usage

Include this in your role to start the Director tree to run ProfileWalk. Edit `ProfileWalk.yaml` to
change the profile, the ramp/hold/rest times or the head position.

The commanded velocity, profile phase and current stage/segment are emitted as `DataPoint`s, so they
can be plotted in NUsight or PlotJuggler alongside the walk engine and sensor data.

## Consumes

- `message::behaviour::state::Stability` to detect when the robot has fallen and when it is back up

## Emits

- `message::behaviour::state::Stability` to set the robot's initial stability state
- `message::behaviour::state::WalkState` to set the robot's initial walk state
- `message::strategy::FallRecovery` to enable getting up when fallen
- `message::skill::Walk` to walk with the profile's current velocity
- `message::skill::Look` to hold the head still for the duration of the profile
- `message::eye::DataPoint` to plot the profile state

## Dependencies

- Director
