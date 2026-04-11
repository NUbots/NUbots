# Mujoco

## Description

MuJoCo simulator integration for NUbots.

It supports two run modes configured in `Mujoco.yaml`:

- `local`: run simulation and offscreen rendering inside the NUbots process/container.
- `remote`: connect to a host-side MuJoCo bridge over TCP and consume simulated sensors from that bridge.

## Usage

Set mode and bridge endpoint in `module/platform/Mujoco/data/config/Mujoco.yaml`:

```yaml
mode: "remote"
remote_host: "172.17.0.1"
remote_port: "18080"
```

Then run the role normally (inside the container):

```bash
./b run mujoco/rl_keyboardwalk
```

When using `local` mode with host X11 display forwarding, the existing display/volume flags can still be used.

## Consumes

## Emits

## Dependencies

- MuJoCo
- GLFW (local mode)
