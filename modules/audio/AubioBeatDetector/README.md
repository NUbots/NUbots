Aubio Beat Detector
===================

## Description

This module finds the beat in music as it plays using the Aubio library's beat
finding algorithm.

## Usage

This module waits for audio input in the form of `message::SoundChunk` objects
and calculates the most likely timing of beats using the Aubio tempo tracking
library.

## Consumes

* `message::SoundChunkSettings` to set up the sample format of upcoming sound
  chunks
* `message::SoundChunk` containing the sound data to find beats in

## Emits

* `message::Beat` every time a beat is detected

## Dependencies

* libaubio is required for tempo detection
* Either the AudioInput or AudioFileInput modules are required to provide sound
  to match beats in

