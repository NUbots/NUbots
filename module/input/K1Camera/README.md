# K1Camera

## Description

Reads shared memory segments created by [NUbridge](https://github.com/NUbots/NUbridge) to read image data and camera info from the Booster K1.

## Usage

`data/config/K1Camera.yaml` contains the following information under `cameras`:
- `segment`: the name of the [Boost](https://www.boost.org/) shared memory segment (usually the name of the ROS2 topic with any slashes being replaced with underscores)
- `id`: camera ID to differentiate between images
- `name`: name of the camera, used to differentiate between outputs

## Emits

- `message::input::Image`

## Dependencies

- [Boost](https://www.boost.org/)
- [NUbridge](https://github.com/NUbots/NUbridge)
- A ROS2 system with the camera topics up and running
