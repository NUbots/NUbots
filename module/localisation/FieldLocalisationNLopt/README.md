# FieldLocalisationNLopt

## Description

A localisation method for estimating the where the field is in world space, which relies on field line points and field line intersections using non-linear optimisation.

### Optimisation Setup

The optimisation framework integrates several cost components and constraints to compute the optimal field localisation state:

### Cost Components

1. **Field Line Alignment Cost ($J_{\text{fl}}$)**:

   - Measures the alignment of predicted field lines with observed ones.
   - Calculated as the squared Euclidean distance between field line points and the nearest point on any observed line, scaled by a predefined weight and divided by the number of field lines:

   $$J_{\text{fl}} = \frac{w_{\text{fl}} \sum_{i=1}^{N} \left( \text{dist}(r_{\text{obs},i}, r_{\text{line}}) \right)^2}{N}$$

   - Points off the field are given a constant weight set in configuration.

1. **Field Line Intersection Cost ($J_{\text{fi}}$)**:

   - Assesses the accuracy of predicted field line intersections against observed intersections.
   - Computed similarly through the squared distances between predicted and observed intersections:

$$J_{\text{fi}} = \frac{w_{\text{fi}} \sum_{j=1}^{M} \left( \text{dist}(r_{\text{int},j}, r_{\text{obs},j}) \right)^2}{M}$$

3. **State Change Cost ($J_{\text{sc}}$)**:
   - Penalizes large deviations from the initial state estimate to ensure temporal consistency.
   - Expressed as:

$$J_{\text{sc}} = w_{\text{sc}} \|\textbf{x} - \textbf{x}\_{\text{init}}\|^2$$

### Constraints

The optimisation is subject to the following constraints:

- **State Bounds**:

  - Limits the allowable state changes between optimisation steps to ensure the solution does not jump an unrealisic amount between updates

$$\textbf{x}\_{\text{init}} - \Delta \mathbf{x} \leq \textbf{x} \leq \textbf{x}\_{\text{init}} + \Delta \textbf{x}$$

- Here, $\Delta \textbf{x}$ represents the maximum allowable change in each state dimension (x, y, and $\theta$).

- **Minimum Field Line Points**:

  - The algorithm requires a minimum number of field line points to run the optimisation to ensure sufficient data for accurate estimation:

$$\text{Count}(\text{field line points}) \geq \text{Min points}$$

- **Robot Stability**:
  - Optimisation will not proceed if the robot is in an unstable state (e.g., falling):

$$\text{stability} > \text{FALLING}$$

### Optimisation Algorithm

- The overall cost function optimized is:

$$J(\textbf{x}) = J_{\text{fl}} + J_{\text{fi}} + J_{\text{sc}}$$

Where:

- $\textbf{x} = [x, y, \theta]^T$ represents the state vector.
- $w_{\text{fl}}$, $w_{\text{fi}}$, and $w_{\text{sc}}$ are weights controlling the relative importance of each cost component.

Optimisation is carried out using NLopt's COBYLA (Constrained Optimisation BY Linear Approximations) algorithm, respecting the constraints and bounds set on the changes allowed in the state to ensure plausible and robust field localisation.

### Cost Threshold and Update Acceptance

The module uses a cost threshold (`cost_threshold`) to determine whether to accept optimization results:

- **Accepting Updates**: Optimization results are only applied if their cost is below `cost_threshold`. This prevents poor localizations from corrupting the state estimate.
- **Rejecting Updates**: When the cost exceeds the threshold, the optimization result is rejected and the previous filtered state is maintained. A warning is logged and a counter is incremented.
- **Triggering Resets**: If the cost exceeds the threshold for `max_over_cost` consecutive updates (and `reset_delay` has elapsed), an uncertainty reset is triggered.

This mechanism provides robustness against temporary vision anomalies or ambiguous field features while maintaining accurate localization when observations are reliable.

### Reset

A soft reset `ResetFieldLocalisation` is used when the robot is starting on the side of the field (or in a custom position specified in the configuration).

A more extreme reset `UncertaintyResetFieldLocalisation` is used when the cost of the current field position is high during play and either a local search or field-wide search must be conducted to regain the position. The local search is a grid search using configurable parameters and uses the lowest cost position if it is under the configurable cost threshold. If this fails to find an appropriate position, a global grid search is conducted. This grid search occurs over the half of the field that the robot was last in, so as to ignore mirror states.

## Usage

Include this module to allow the robot to estimate where the field is in world space.

## Consumes

- `message::vision::FieldLines` field line points are used in the field localisation cost function
- `message::vision::FieldLineIntersections` field line intersections are used in the field localisation cost function
- `message::vision::Goals` goal positions are used in the field localisation cost function
- `message::support::FieldDescription` to determine the field map and bounds of the field

## Emits

- `message::localisation::Field` contains the estimated (x, y, theta) state
- `message::localisation::ResetFieldLocalisation` signalling a side-of-field localisation reset
- `message::localisation::UncertaintyResetFieldLocalisation` signalling a local or field-wide reset, which is a computationally intensive action
- `message::localisation::FinishReset` signalling that a local or field-wide reset has completed

## Dependencies

- `Eigen`
- `utility::math::stats::MultivariateNormal` Utility for sampling from a multivariate normal distribution
