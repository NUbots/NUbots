Beat Detectors Tests
====================

## Description

Tests a beat detector by recording the timing of beats that it detectsi to a
file.

## Usage

BeatDetectorsTests waits for a `messages::SoundFileStart` message to indicate
that a sound file has been loaded, then opens a similarly named text file. At
each `messages::Beat`, it records the time since the start of the sound into the
text file.

## Consumes

* `messages::SoundFileStart` to indicate the start of a new sound file
* `messages::Beat` indicating a beat has been detected

## Dependencies

* AudioFileInputArgs is used to read the audio file, no other input module
  emits `messages::SoundFileStart`
* A beat detection module is required

