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
   - Penalises large deviations from the initial state estimate to ensure temporal consistency.
   - Expressed as:

$$J_{\text{sc}} = w_{\text{sc}} \|\textbf{x} - \textbf{x}\_{\text{init}}\|^2$$

   - `state_change_weight` ($w_{\text{sc}}$) now defaults to `0.0`: temporal consistency is instead provided by
     the probabilistic filter's covariance prior (see "Probabilistic Filter and Validity Gating" below). The
     term is kept as an escape hatch for degenerate geometry (e.g. sliding along a single visible line) where
     the filter's flat-direction covariance clamp isn't sufficient on its own. Critically, this term is only
     ever added on top of `evaluate_cost`'s perception-only terms inside `run_field_line_optimisation` - the
     Hessian and validity metric always call `evaluate_cost` directly and never see it, since it would inject
     artificial curvature exactly in the directions the filter needs to identify as unobserved.

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

- The overall cost function optimised is:

$$J(\textbf{x}) = J_{\text{fl}} + J_{\text{fi}} + J_{\text{sc}}$$

Where:

- $\textbf{x} = [x, y, \theta]^T$ represents the state vector.
- $w_{\text{fl}}$, $w_{\text{fi}}$, and $w_{\text{sc}}$ are weights controlling the relative importance of each cost component.

Optimisation is carried out using NLopt's COBYLA (Constrained Optimisation BY Linear Approximations) algorithm, respecting the constraints and bounds set on the changes allowed in the state to ensure plausible and robust field localisation.

### Probabilistic Filter and Validity Gating

The raw NLopt optimisation result is treated as a **measurement** of the (x, y, theta) state, fused into a
hand-rolled 3-state Kalman filter (`localisation/PoseFilter.hpp`) rather than being smoothed directly with an
exponential filter:

- **Process model**: the state parameterises `Hfw`, which is constant under perfect odometry, so the filter
  uses an identity transition. The `predict` step instead grows the covariance in proportion to observed
  odometry motion (`||delta translation|| + |delta yaw|` between consecutive `Sensors::Hrw`), scaled by
  `kalman.process_noise`.
- **Measurement covariance from curvature**: after each accepted optimisation, a central-difference Hessian of
  the perception-only cost (`evaluate_cost`, field-line + intersection + goal-post terms, deliberately
  excluding the state-change regulariser) is computed at the optimum (`finite_difference_hessian`,
  `localisation/filter.cpp`). Its eigenvalues are mapped to per-direction measurement variances via
  `covariance_from_hessian`: `R_i = clamp(kalman.measurement_scale / lambda_i, r_min, r_max)`, with
  non-positive or near-singular curvature (an unobserved/unreliable direction, e.g. along a single visible
  line) clamped to `r_max` rather than falling back to a fixed covariance for the whole update. This lets
  degenerate geometry inflate uncertainty in just the unobserved direction instead of everywhere.
- **Update**: the filter's Kalman gain fuses the measurement and its covariance into the state estimate; the
  heading innovation is angle-wrapped (`utility::math::angle::signedDifference`) so it doesn't fight the +-pi
  seam.
- **Search box**: each frame's optimisation is seeded from the filter's mean and bounded by a box derived from
  the filter's current uncertainty (`clamp(3*sigma, 0.1, change_limit)` per axis) rather than always using the
  full `change_limit` - this tightens the search once the filter is confident and widens it again after a
  period of poor observations.

Acceptance is gated on a **validity** metric (`compute_validity`) rather than raw cost, since cost magnitudes
are not directly comparable across differing numbers/mixes of observations: validity is the fraction of
field-line points registered within `validity.line_inlier_distance` of the field-line map, blended 50/50 with
the fraction of field intersections successfully associated with a landmark (when intersections are observed;
lines-only otherwise). Optimisation results with `validity >= validity.min_validity` are fused into the
filter; results below threshold are rejected (the filter's predict step still ran, so uncertainty keeps
growing) and a counter of consecutive invalid frames is incremented.

The `field->cost` emitted in the `Field` message is **unchanged in meaning** - it is still the raw perception
cost of the accepted/proposed hypothesis - specifically so that downstream consumers gating on
`field.cost > max_localisation_cost` (`StrategiseLook`, `FieldPlayer`) continue to behave the same way; only
the *internal* accept/reject decision moved from cost to validity. The emitted `field->covariance` and
`field->uncertainty` (`trace(cov)`) are now populated from the filter for the first time; they were previously
always zero.

- **Triggering Resets**: If validity stays below `validity.min_validity` for more than
  `validity.max_invalid_frames` consecutive updates (and `reset_delay` has elapsed since the last reset), an
  uncertainty reset is triggered, replacing the old cost-threshold-based `reset_on_cost`/`max_over_cost`
  mechanism.

### Reset

A soft reset `ResetFieldLocalisation` is used when the robot is starting on the side of the field (or in a
custom position specified in the configuration); this also resets the pose filter to `change_limit`
uncertainty around the chosen initial hypothesis.

A more extreme reset `UncertaintyResetFieldLocalisation` is used when validity has been too low for too long
during play. This now runs a staged pipeline (`localisation/candidates.cpp`, `localisation/reset.cpp`):

1. **Analytic candidates** (`reset.use_candidates`): generate one-shot candidate poses without any search -
   `goal_pair_candidates` from the two observed goal posts (both assignment orders, both field ends, gated by
   the expected goal-post distance), and `intersection_pair_candidates` (HTWK-style) from every pair of
   observed field intersections matched against model landmark pairs of the same types and similar separation
   (`reset.pair_separation_tolerance`). Mirror twins arise automatically from the symmetric landmark layout.
   Candidates are deduplicated (within 0.3 m / 0.3 rad) and capped at `reset.max_candidates`, preferring
   widely-separated generating pairs (better angular conditioning). `last_certain_state` is always included as
   a candidate.
2. **Short refine**: each candidate is refined with a short SBPLX optimisation and ranked by validity; the best
   is accepted (and the filter hard-reset to it) if its validity clears `validity.min_validity`.
3. **Grid-search fallback**: if no candidate is good enough (or candidate generation is disabled/empty), a
   local-window grid search around `last_certain_state` is tried first, followed by a wider search if that
   also fails, both now gated on validity instead of raw cost.
4. On acceptance at any stage, `filter.reset(state, change_limit)` re-initialises the pose filter's mean and
   covariance to the accepted candidate.

`reset.constrain_to_half` (default `true`) preserves the original behaviour of only ever searching/considering
the half of the field the robot was last on during the grid-search fallback (and never adding mirrored
candidates in stage 1), avoiding the mirror field problem structurally. See "Mirror Symmetry Limitation" below
for why this is a structural constraint rather than a per-frame evidence check.

### Mirror Symmetry Limitation

The field localisation cost function is **exactly invariant** under the mirror transform
`(x, y, theta) -> (-x, -y, theta + pi)`: the field-line distance map, the typed landmark pairs used for
intersection association, and the sign-selected goal-post term (`rGFf_left.x() > 0 ? own_goal_posts :
opp_goal_posts`) are all symmetric about the field centre, so `cost(mirror(s)) === cost(s)` for every state
`s`. The same is true of the validity metric, since it is built from the same distance map and association
logic.

**Consequence**: vision alone can never disambiguate which half of the field the robot is actually on - a
per-frame "evidence ratio" comparing `cost(s)` against `cost(mirror(s))` would be identically 1 for every
frame and is deliberately **not implemented**. Instead this module relies on structural mitigations:

- `reset.constrain_to_half` (default `true`) never lets a reset search consider the wrong half of the field in
  the first place.
- When `reset.constrain_to_half` is `false`, mirror twins are allowed to compete during a reset, and ties are
  broken explicitly by distance (position + wrapped angle) to `last_certain_state` - see
  `is_mirror_twin`/`pose_distance` in `localisation/reset.cpp`.
- The `PenaltyReset` handler logs a `WARN` ("Mirror flip detected via penalty reset") when the mirror image of
  the current filter estimate is closer to the known penalty-kick position than the current estimate itself -
  one of the few moments an external positional cue is available. `mirror.auto_flip` (default `false`) gates
  whether this cue is acted on automatically (flipping the filter's mean) or only logged; it is an extension
  point for future GameController-sourced evidence (e.g. kickoff/half-time side swaps), not wired up in this
  change.

### Deferred: Single-Intersection and Line-Direction Candidates

`intersection_pair_candidates` requires *pairs* of observed intersections because a single intersection's
position alone doesn't fix orientation - `message::vision::FieldIntersection` only carries `{type, rIWw}`, with
no local line direction. Adding single-intersection candidates (matching one observed intersection plus its
line direction against a single model landmark) would need a vision-message change to carry that direction,
which is out of scope for this change. This is deferred alongside upcoming extrinsics/vision work.

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
