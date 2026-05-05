# UncertaintyResetTester

## Description

Times each `UncertaintyResetFieldLocalisation` → `FinishReset` round-trip emitted by
`localisation::FieldLocalisationNLopt` while an NBS recording is replayed. Reset
durations are logged, graphed (so they show up in NUsight) and summarised at the end of
playback.

Optionally, the module can periodically emit a `PenaltyReset` to a deliberately bad pose
to provoke cost-driven uncertainty resets, which is useful when a recording does not
naturally trigger them.

## Usage

```bash
./b run test/uncertainty_reset <path_to_data.nbs>
```

The recording must contain at minimum `RawSensors`, `FieldLines`, `FieldIntersections`
and `Goals` messages.

## Configuration

See `data/config/UncertaintyResetTester.yaml`. Set `force_reset_period > 0` to provoke
resets at a fixed cadence; tune `force_reset_position` to a pose that is far from the
true robot path in the recording.
