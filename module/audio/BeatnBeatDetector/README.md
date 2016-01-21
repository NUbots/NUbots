Beat'n Beat Detector
============

## Description

This module finds the beat in music as it plays.

## Usage

This module waits for audio input in the form of `message::SoundChunk` objects
and calculates the most likely timing of beats based on a sliding ten-second
window of sound data. Whenever a beat is detected it emits a `message::Beat`
containing the current time and the beat's frequency.

If the input audio has multiple channels, only the first channel is analysed.

## Consumes

* `message::SoundChunkSettings` to set up the sample format of upcoming sound
  chunks
* `message::SoundChunk` containing the sound data to find beats in

## Emits

* `message::Beat` every time a beat is detected

## Dependencies

* libfftw3 is used to perform FFTs
* Either the AudioInput or AudioFileInput modules are required to provide sound
  to match beats in

