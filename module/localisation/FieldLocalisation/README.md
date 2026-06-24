# FieldLocalisation

## Description
This module provides a robust, probabilistic Particle Filter (Monte Carlo Localisation) to estimate the robot's pose on the soccer field. It tracks the transformation from the odometry-drifting `world` frame to the absolute `field` frame (`Hfw`).

### Why Particle Filter?
This module replaces older point-estimate/optimization-based localisation (like NLopt) to solve several inherent problems:
1. **Multimodal Tracking (Kidnapping):** By maintaining a large set of particles, the filter can simultaneously track multiple hypotheses. If the robot gets "kidnapped" or turned around, it evaluates both sides of the symmetric field without getting permanently stuck in a local minimum.
2. **Robustness to Outliers:** The measurement model uses likelihood fields with heavy-tailed distributions (combining Gaussian peaks around map features with a uniform random probability). This prevents stray visual noise (e.g., false field lines or misclassified intersections) from corrupting the localisation.
3. **No Expensive Resets:** The module continuously injects a small percentage of random particles across the field. This completely removes the need for catastrophic global grid searches that spike the CPU, as the filter inherently "discovers" its true location over time.
4. **Proper Uncertainty Estimation:** Computes the true covariance matrix and trace (uncertainty) of the particle cloud, allowing downstream behaviours to make intelligent decisions based on how confident the robot is in its pose.

## Configuration
- `num_particles`: The total number of particles to use. (Default: 1000)
- `random_particle_injection_rate`: Proportion of particles replaced with random uniform samples at each resampling step to handle kidnapped-robot problems.
- `process_noise`: Noise coefficients applied during odometry updates to account for drift.
- `*_sigma`: Measurement likelihood sensitivities for field lines, intersections, and goal posts.

## Dependencies
- `Sensors`: Provides the `Hrw` odometry updates.
- `FieldLines`, `FieldIntersections`, `Goals`: Used to compute particle likelihoods against the map.
- `FieldDescription`: Generates the internal `OccupancyMap`.

## Emits
- `message::localisation::Field`: Contains the `Hfw` transform, covariance, uncertainty score, and particle cloud.
