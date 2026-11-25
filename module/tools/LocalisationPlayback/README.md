# LocalisationPlayback

## Description

Plays a recording's sensors and vision through whichever field localisation module is in the role, and writes every
`message::localisation::Field` it emits to a csv as the torso's pose in the field (`Hft = Hfw * Htw^-1`, against the
latest `Sensors`): `t_ns, x, y, yaw, localised, uncertainty, cost`. `t_ns` is the recording's time (ns since the
epoch), as the Player time travels NUClear's clock along it.

The csv is `<output_directory>/<recording>_<binary>.csv`, so `playback/localisation_nlopt` and
`playback/localisation_srif` write side by side.

## Usage

```bash
./b localisation playback recordings/<recording>.nbs
```

runs both roles, NLopt then SRIF, and plots them against the recording's ground truth into
`recordings/<recording>_localisation.png` (csvs in `recordings/localisation_playback/`). Pass
`--config-hostname <robot hostname>` (or `docker`) for real robot recordings so the right field is loaded; the default,
`nusim`, loads NUSim's.

A single role can also be run on its own: `./b run playback/localisation_srif recordings/<recording>.nbs`.

The recording needs (DataLogging.yaml):

```yaml
message.input.Sensors: true # both
message.vision.FieldIntersections: true # NLopt
message.vision.Goals: true # NLopt
message.vision.BoundingBoxes: true # SRIF
# Ground truth: input::NatNet + localisation::Mocap on a real robot, input::NUSimGroundTruth (ground_truth: true) in NUSim
message.localisation.RobotPoseGroundTruth: true
```

## Consumes

- `message::localisation::Field`
- `message::input::Sensors`
- `message::nbs::player::PlaybackState`, `PlaybackFinished`

## Emits

- `message::nbs::player::SetModeRequest`, `LoadRequest`, `PlayRequest`

## Dependencies

- `nbs::Player`
