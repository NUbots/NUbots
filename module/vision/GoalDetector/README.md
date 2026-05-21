# GoalDetector

## Description

Detects goal posts from `GreenHorizon` + VisualMesh classifications and publishes goal observations.

The detector:

- Selects VisualMesh points classified as `goal` above a confidence threshold
- Clusters candidate points and keeps clusters intersecting the green horizon
- Merges overlapping clusters
- Estimates goal-post bottom/top rays and distance in camera space
- Pairs posts as left/right using expected field goal width and a disagreement threshold

## Usage

Include this module after `GreenHorizonDetector`/VisualMesh classification.

Configuration (`GoalDetector.yaml`):

- `confidence_threshold`: minimum goal-class confidence per point
- `cluster_points`: minimum cluster size to consider as a goal post
- `disagreement_ratio`: tolerated relative error between measured post-pair width and configured field goal width

The module triggers on `message::vision::GreenHorizon` and requires `message::support::FieldDescription` for goal
geometry (`goal_width`, `goalpost_top_height`).

## Consumes

- `extension::Configuration` from `GoalDetector.yaml`
- `message::vision::GreenHorizon`
- `message::support::FieldDescription`

## Emits

- `message::vision::Goals`

## Dependencies

- VisualMesh clustering utilities (`partition_points`, `cluster_points`)
- Green horizon filtering utilities (cluster-horizon intersection checks)
