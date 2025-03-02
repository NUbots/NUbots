# LocalisationBenchmark

## Description

This module benchmarks the performance of the robot's field localisation system by comparing its estimates against ground truth data during NBS file playback. It calculates Root Mean Square Error (RMSE) for translation and rotation errors, as well as individual errors for each degree of freedom (x, y, z, roll, pitch, yaw).

## Usage

1. **Run with NBS File**:
   Execute the module by providing an NBS file containing ground truth and sensor data as a command-line argument:
   ```bash
   ./b run localisation_benchmark <path_to_data.nbs>
   ```
2. **Required NBS Messages**:
   Ensure the NBS file includes the following messages:
   ```yaml
   message.platform.RawSensors: true # Ground truth odometry and localisation
   message.vision.Goals: true # Goal post detections
   message.vision.FieldLines: true # Field line detections
   message.vision.FieldIntersections: true # Field intersection detections
   ```
3. **Parameter Optimization**:
   Use the `optimize_localisation` script to tune sensor filter and localisation parameters using a genetic algorithm:
   ```bash
   ./b optimize_localisation <ground_truth_data.nbs>
   ```

## Dependencies

- `message::platform::RawSensors` for ground truth comparison and odometry.
- `message::vision::Goals`
- `message::vision::FieldLines`
- `message::vision::FieldIntersections`
- NUbots playback system for NBS file handling.
