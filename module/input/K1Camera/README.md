# K1Camera

## Description

Subscribes directly to the Booster K1's camera topics using the [Booster Robotics
SDK](https://github.com/BoosterRobotics/booster_robotics_sdk) (1.7) over DDS, and emits the frames as
`message::input::Image`.
## Usage

`data/config/K1Camera.yaml` contains the following information under `cameras`:

- `topic`: the DDS topic carrying `sensor_msgs/Image`. This is the ROS 2 topic path prefixed with
  `rt/`, so ROS 2 topic `/boostercamera/head/rgb` becomes `rt/boostercamera/head/rgb`
- `name`: name of the camera, used to differentiate between outputs
- `id`: camera ID to differentiate between images

Topics are subscribed to once at startup. Changing `topic` in the config while running will not
re-point the subscriptions; restart the binary instead.

## Consumes

- `message::input::Sensors` to buffer recent `Hcw` transforms

## Emits

- `message::input::Image`

## Dependencies

- [Booster Robotics SDK](https://github.com/BoosterRobotics/booster_robotics_sdk)
- [OpenCV](https://opencv.org/)
- A running BoosterOS publishing the configured camera topics
