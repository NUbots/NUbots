# OdometryBenchmark

## Description

This module benchmarks the performance of the robot's odometry system (specifically `Htw` from `SensorFilter`) by comparing its estimates against ground truth data during NBS file playback. It calculates Root Mean Square Error (RMSE) for translation and rotation errors, as well as individual errors for each degree of freedom (x, y, z, roll, pitch, yaw).

## Usage

1.  **Run with NBS File**:
    Execute the module by providing an NBS file containing ground truth and sensor data as a command-line argument:
    ```bash
    ./b run webots/odometry_benchmark <path_to_data.nbs>
    ```
2.  **Required NBS Messages**:
    Ensure the NBS file includes the following messages:
    ```yaml
    message.platform.RawSensors: true # Contains odometry ground truth (Htw)
    ```
3.  **Parameter Optimisation**:
    Use the `optimise_odometry` script (to be created) to tune sensor filter parameters:
    ```bash
    ./b optimise_odometry --nbs_file <ground_truth_data.nbs> --n_trials 500 --hostname webots
    ```

## Dependencies

-   `message::platform::RawSensors` for ground truth odometry comparison.
-   `message::input::Sensors` for the estimated odometry.
-   NUbots playback system for NBS file handling.
