# NUSimCamera

## Description

Reads the camera image out of the shared-memory segment written by **NUSim** (the
MuJoCo simulator's `Camera` module / `SharedImageWriter`) and emits it as
`message::input::Image`. NUSimCamera reads NUSim's VERSION 2 **lock-free seqlock** layout.

Use this module in `roles/nusim/*`; use `input::K1Camera` in the on-robot roles.

## Usage

`data/config/NUSimCamera.yaml` lists, under `cameras`:
- `segment`: name of the [Boost](https://www.boost.org/) shared-memory segment NUSim writes
- `id`: camera ID to differentiate between images
- `name`: name of the camera, used to differentiate between outputs

## Emits

- `message::input::Image`

## Dependencies

- [Boost](https://www.boost.org/)
- NUSim writing the camera shared-memory segment (`SharedImageWriter`, VERSION 2 seqlock)
