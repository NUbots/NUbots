# PS3Walk

## Description

Starts the Director graph for manual teleoperation using a PS3 controller.

`PS3Walk` reads joystick events from `/dev/input/js0` and publishes walk, look, kick, and optional script tasks.
It also applies acceleration limiting to walk commands for smoother velocity changes.

## Usage

Include this module in a role and connect a PS3-compatible joystick device.

Key behaviour:

- `START`: toggle walking on/off
- Left stick: forward/back + rotation command
- `SELECT`: toggle head lock/unlock
- Right stick (when unlocked): control head yaw/pitch
- Right stick button: toggle script execution on/off
- Face/DPAD/L1/R1 buttons: run mapped scripts or configured kick actions (when scripts are enabled)

Default script/kick mapping is configured in `PS3Walk.yaml` via `button_scripts`.

### Configuration

- `maximum_forward_velocity`: max forward velocity command (m/s)
- `maximum_rotational_velocity`: max rotational velocity command (rad/s)
- `max_acceleration`: max change in velocity per second
- `button_scripts`: map button names to script files or kick actions (`LEFT_LEG_KICK`, `RIGHT_LEG_KICK`)

## Consumes

- `extension::Configuration` from `PS3Walk.yaml`
- Joystick device events from Linux input (`/dev/input/js0`)

## Emits

- `message::behaviour::state::Stability` (initialised to `UNKNOWN` on startup)
- `message::behaviour::state::WalkState` (initialised to `STOPPED` on startup)
- `message::skill::Look` (head control)
- `message::skill::Walk` (teleoperation walk command)
- `message::skill::Kick` (when `L1`/`R1` map to kick actions)
- `message::actuation::LimbsSequence` (for configured button scripts)

## Dependencies

- Linux joystick device interface (`/dev/input/js*`)
