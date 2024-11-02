# FieldLocalisationNLopt

## Description

A localisation method for estimating the where the field is in world space, which relies on field line points and field line intersections using non-linear optimisation.

### Optimization Setup

The optimization framework integrates several cost components and constraints to compute the optimal field localisation state:

### Cost Components

1. **Field Line Intersection Cost ($J_{\text{fi}}$)**:

   - Assesses the accuracy of predicted field line intersections against observed intersections.
   - Computed similarly through the squared distances between predicted and observed intersections:
     $$ J*{\text{fi}} = w*{\text{fi}} \sum*{j=1}^{M} \left( \text{dist}(r*{\text{int}_j}, r_{\text{obs}\_j}) \right)^2 $$

2. **State Change Cost ($J_{\text{sc}}$)**:

   - Penalizes large deviations from the initial state estimate to ensure temporal consistency.
   - Expressed as:
     $$ J*{\text{sc}} = w*{\text{sc}} \|\textbf{x} - \textbf{x}\_{\text{init}}\|^2 $$

3. **Goal Post Cost (J\*{\text{g}})**:

   - Assesses the accuracy of goal post observations with known positions
   - $$ J*{\text{g}} = w*{\text{fi}} \sum*{j=1}^{M} \left( \text{dist}(r*{\text{int}_j}, r_{\text{obs}\_j}) \right)^2 $$

### Constraints

The optimization is subject to the following constraints:

- **State Bounds**:

  - Limits the allowable state changes between optimization steps to ensure the solution does not jump an unrealisic amount between updates
    $$ \textbf{x}_{\text{init}} - \Delta \textbf{x} \leq \textbf{x} \leq \textbf{x}_{\text{init}} + \Delta \textbf{x} $$
  - Here, $\Delta \textbf{x}$ represents the maximum allowable change in each state dimension (x, y, and $\theta$).

- **Robot Stability**:
  - Optimization will not proceed if the robot is in an unstable state (e.g., falling):
    $$ \text{stability} > \text{FALLING} $$

### Optimization Algorithm

- The overall cost function optimized is:
  $$ J(\textbf{x}) = J*{\text{g}} + J*{\text{fi}} + J\_{\text{sc}} $$

Where:

- $\textbf{x} = [x, y, \theta]^T$ represents the state vector.
- $w_{\text{fl}}$, $w_{\text{fi}}$, and $w_{\text{sc}}$ are weights controlling the relative importance of each cost component.

Optimization is carried out using NLopt's COBYLA (Constrained Optimization BY Linear Approximations) algorithm, respecting the constraints and bounds set on the changes allowed in the state to ensure plausible and robust field localisation.

## Usage

Include this module to allow the robot to estimate where the field is in world space.

## Consumes

- `message::vision::FieldLines` uses the field line observations from FieldLineDetector module
- `message::vision::FieldLineIntersections` uses the field line intersections from FieldLineDetector module

## Emits

- `message::localisation::Field` contains the estimated (x, y, theta) state

## Dependencies

- `Eigen`
- `utility::math::stats::MultivariateNormal` Utility for sampling from a multivariate normal distribution
