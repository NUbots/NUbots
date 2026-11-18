# K1 Stereo Benchmark

A hardware benchmarking framework for testing **Fast-FoundationStereo (FFS)** on the NVIDIA Orin NX and D-Robotics RDK X5.

**Goal:** Investigate the feasibility of real-time stereo depth estimation on both platforms by comparing inference performance, latency and resource utilisation.

## TODO

- [ ] Build containerised perception module
- [x] Identify K1 raw stereo camera streams
- [ ] Record synchronised and rectified left/right camera pairs
- [ ] Run FFS on Orin NX
- [ ] Investigate deployment on RDK X5
- [ ] Compare inference performance across both platforms
- [ ] Evaluate hardware acceleration using TensorRT and Horizon BPU

## Project Structure

```text
stereo-benchmark/
├── docker/
│   ├── orin/
│   │   └── Dockerfile
│   └── rdk/
│       └── Dockerfile
├── src/
│   ├── camera/       # Stereo camera acquisition
│   ├── stereo/       # Model integration and preprocessing
│   ├── backends/     # Hardware-specific inference
│   └── benchmark/    # Performance measurement
├── config/           # Platform configurations
├── scripts/          # Setup and deployment
├── datasets/         # Recorded stereo image pairs
├── models/           # Model weights and converted engines
├── results/          # Benchmark results
├── docker-compose.yml
└── README.md
```

## K1 Stereo Camera

The Booster K1 uses the RDK X5 camera pipeline, exposing separate stereo streams through ROS 2 using the `mipi_cam` driver.

| Property | Left Camera | Right Camera |
|---|---|---|
| ROS 2 topic | `/boostercamera/head/raw/rgb` | `/boostercamera/head/raw/right/rgb` |
| Encoding | NV12 | NV12 |
| Resolution | 544 × 448 | 544 × 448 |
| Row step | 544 bytes | 544 bytes |
| Measured FPS | ~29.8 | ~30.0 |

Camera calibration is available through the corresponding `camera_info` topics.

The existing StereoNet pipeline also publishes rectified images and depth.

### ROS 2 Camera Topics

| Topic | Message Type | Description |
|---|---|---|
| `/boostercamera/head/raw/rgb` | `sensor_msgs/msg/Image` | Raw left camera image |
| `/boostercamera/head/raw/right/rgb` | `sensor_msgs/msg/Image` | Raw right camera image |
| `/boostercamera/head/raw/combine/rgb` | `sensor_msgs/msg/Image` | Combined stereo image |
| `/boostercamera/head/raw/rgb/camera_info` | `sensor_msgs/msg/CameraInfo` | Raw left camera calibration |
| `/boostercamera/head/raw/right/rgb/camera_info` | `sensor_msgs/msg/CameraInfo` | Raw right camera calibration |
| `/boostercamera/head/rgb` | `sensor_msgs/msg/Image` | Rectified left image |
| `/boostercamera/head/right/rgb` | `sensor_msgs/msg/Image` | Rectified right image |
| `/boostercamera/head/rgb/camera_info` | `sensor_msgs/msg/CameraInfo` | Rectified left camera calibration |
| `/boostercamera/head/depth` | `sensor_msgs/msg/Image` | StereoNet depth output |
| `/boostercamera/head/depth/camera_info` | `sensor_msgs/msg/CameraInfo` | Depth camera information |

Raw image streams are published by `mipi_cam`. Rectified images and depth are published by the existing StereoNet pipeline.

**Benchmark acquisition sources:**
- `/boostercamera/head/raw/rgb`
- `/boostercamera/head/raw/right/rgb`

The raw streams will be used as the initial recording sources. Stereo synchronisation and rectification still need to be verified before creating the benchmark dataset.

## Software and Deployment

Docker will provide reproducible software environments, with separate images for hardware-specific dependencies.

### NVIDIA Jetson Orin NX

- NVIDIA JetPack and CUDA
- PyTorch
- TensorRT
- NVIDIA Container Toolkit

### D-Robotics RDK X5

- D-Robotics Horizon software stack
- Horizon BPU runtime
- Compatible model conversion tools

Actual dependencies and versions will be confirmed against the installed platform environments.

## Benchmarking Methodology

The benchmark will measure the following metrics:

| Metric | Description |
|---|---|
| Inference FPS | Stereo inference throughput |
| Average latency | Average inference time per stereo pair |
| Median latency | Median inference time |
| P95 latency | 95th percentile inference latency |
| Peak memory | Maximum memory consumption |
| CPU utilisation | CPU resource consumption |
| Accelerator utilisation | GPU/BPU utilisation |
| Power consumption | Platform power usage |

Inference-only performance and end-to-end performance (capture, preprocessing and inference) will be measured separately.

Both platforms should process identical recorded stereo image pairs wherever possible to ensure a fair comparison.
