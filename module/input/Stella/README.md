# Stella

## Description
The Stella module is a NUClear reactor that integrates Stella VSLAM into the NUbots system. It provides real-time SLAM capabilities for robot localization and mapping using monocular camera input.

The module processes incoming camera images, feeds them to the Stella VSLAM system for feature detection and tracking, and provides both debug visualization and real-time mapping data through a socket publisher interface.

## Usage
Run the following commands to copy libsocket_publisher.so to the robot:
scp libsocket_publisher.so nubots@10.1.1.1:/usr/local/lib/
Run ./test/stella on robot:
 - Visualizer in NUsight both the normal camera feed + Stella camera feed (with feature detections).

SocketViewer:
- Run the following commands to stop libuv from getting in the way:
sudo mkdir -p /usr/local/lib. bak_libuv
sudo sh -c 'mv /usr/local/lib/libuv.so* /usr/local/lib/.bak_libuv/ 2>/dev/null || true'
- Navigate to directory socket_viewer on the robot, and run 'npm install' to verify.
- Run node app.js to start socket viewer.
- Enter 10.1.1.1:3001 into google to open SocketViewer.
- Run ./test/stella.

## Consumes


## Emits


## Dependencies

## TODO:
- Apply extrinsic offset to stella result
- Solve the translation scaling problem
- Setup Mocap system to record ground truth dataset and compare again Stella result
- Add Stella map viewer in NUsight
- Fuse with IMU/Kinematic Odometry
