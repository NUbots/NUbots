# WhistleDetection

## Description

Detects a referee whistle in real-time from the robot's microphone and emits a
`message::input::Whistle` message when one is confirmed.

The algorithm mirrors the signal-processing front-end used by the NaoDevils
[AudioProcessing](https://github.com/NaoDevils/AudioProcessing) project (from
which the labelled dataset was obtained):

1. Capture mono audio via ALSA at 16 kHz.
2. Accumulate samples into 1024-point windows with 50 % overlap (one new window
   every 32 ms).
3. Apply a Hanning window and compute the one-sided power spectrum via a
   radix-2 Cooley-Tukey FFT.
4. Compute the fraction of total spectral power that falls in the configurable
   whistle frequency band (default 2–4.5 kHz).
5. Require `confirm_frames` consecutive windows above `band_ratio_threshold` before
   emitting, and enforce a `cooldown_ms` gap between successive events.

## Usage

Add the module to a role file and tune `WhistleDetection.yaml` for the deployment
environment (primarily `band_ratio_threshold` and `min_band_energy`, which depend
on microphone gain and background noise level).

## Consumes

Nothing — audio is read directly from the ALSA device.

## Emits

- `message::input::Whistle` — fired once per confirmed whistle with a timestamp
  and a confidence value in [0, 1].

## Dependencies

- ALSA (`libasound`) — already a project-wide utility dependency.
