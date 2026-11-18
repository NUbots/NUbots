# K1 Stereo Benchmark

A hardware benchmarking framework for testing ffs on the Orin NX vs the RDK x5.

Goal: investigate the feasibility of running real-time stereo depth estimation on both platforms. Compare inference performance, latency, resource utils.

TODO:

- Build containerised perception module
- Access raw stereo camera data from K1 (in this branch)
- Retreived synced and rectified L/R camera streams
- Run ffs on Orin NX
- Investigate deployment on RDK x5
- compare inference performance across both computing platforms
- evaluate hardware acceleration using trt and Horizon BPU runtime

```
stereo-benchmark/
│
├── docker/
│   ├── orin/
│   │   └── Dockerfile
│   └── rdk/
│       └── Dockerfile
│
├── src/
│   ├── camera/       # Stereo camera acquisition
│   ├── stereo/       # Model integration and preprocessing
│   ├── backends/     # Hardware-specific inference backends
│   └── benchmark/    # Performance measurement and reporting
│
├── config/           # Platform-specific configurations
├── scripts/          # Setup and deployment utilities
├── datasets/         # Recorded stereo image pairs
├── models/           # Model weights and converted engines
├── results/          # Benchmark outputs
│
├── docker-compose.yml
└── README.md

```

stereo-benchmark/
│
├── docker/
│   ├── orin/
│   │   └── Dockerfile
│   └── rdk/
│       └── Dockerfile
│
├── src/
│   ├── camera/       # Stereo camera acquisition
│   ├── stereo/       # Model integration and preprocessing
│   ├── backends/     # Hardware-specific inference backends
│   └── benchmark/    # Performance measurement and reporting
│
├── config/           # Platform-specific configurations
├── scripts/          # Setup and deployment utilities
├── datasets/         # Recorded stereo image pairs
├── models/           # Model weights and converted engines
├── results/          # Benchmark outputs
│
├── docker-compose.yml
└── README.md

Software and Deployment

The application is designed to use Docker to maintain reproducible software environments across both computing platforms.

Hardware-specific dependencies are isolated into separate container images.

NVIDIA Jetson Orin NX

Expected software components:

NVIDIA JetPack

CUDA

PyTorch

TensorRT

NVIDIA Container Toolkit

D-Robotics RDK X5

Expected software components:

D-Robotics Horizon software stack

Horizon BPU runtime

Compatible model conversion tools

Required hardware device interfaces

Actual dependencies and versions will be determined from the installed software environments.

Benchmarking Methodology

The benchmark will measure:

Metric

Description

Inference FPS

Stereo inference throughput

Average latency

Average inference time per stereo pair

Median latency

Median inference time

P95 latency

95th percentile inference latency

Peak memory

Maximum memory consumption

CPU utilisation

CPU resource consumption

Accelerator utilisation

GPU/BPU utilisation

Power consumption

Platform power usage
