NUsight
========

## Description

This module allows for more effective debugging of the robocup program by
sending the robot's status over the network.

## Usage

The NUsight module listens on TCP port 12000, which clients on remote machines
running NUsight can connect to in order to receive and monitor data from the
robot in real time. Information is serialised using the NUAPI Message protocol
buffer.

There are two message types currently implemented: `SENSOR_DATA` and `VISION`.
`SENSOR_DATA` contains the status of each servo motor, the gyro and
accelerometer and the current orientation. `VISION` contains a JPEG-compressed
copy of an image from the robot's camera.

## Consumes

* `message::DarwinSensors` containing sensor data
* `message::Image` containing a frame from the camera

## Dependencies

* libzmq is used for network communication
* libprotobuf is used for data serialization
* The Darwin HardwareIO module must be installed to report sensor data
* The LinuxCameraStreamer module must be installed to transmit images
* libjpeg is used to compress camera images

