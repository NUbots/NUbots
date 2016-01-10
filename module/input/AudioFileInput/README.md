Audio File Input
================

## Description

Reads sound from a file and emits it in regular-sized chunks as if it were
being recorded in real time.

## Usage

AudioFileInput opens the audio file specified in its configuration file. It
then emits a `message::SoundChunkSettings` containing the sound file's sample
rate, bit depth and the number of samples that will be in each sound chunk.

Once running, the module regularly emits buffers of audio samples in
`message::SoundChunk` objects. The number of chunks to emit per second is set
by the `CHUNKS_PER_SECOND` constant, which is currently set to 100 (producing
100 10ms chunks per second).

Sound chunks are created in real time - five minutes worth of audio in the
source file will be emitted in chunks over a five minute period.

This module and AudioInput both emit audio data using the same messages, as
such it is possible to use AudioFileInput to effectively emulate microphone
input with pre-recorded sound.

## Consumes

* `message::Configuration<AudioFileConfiguration>` from the config system to
  set the name of the audio file to read

## Emits

* `message::SoundChunk` containing audio samples read from the file
* `message::SoundChunkSettings` to describe the sample format of upcoming
  sound chunks

## Configuration

This module's configuration is stored in AudioFileInput.yaml. There is only one
setting, which is required.

* "file": the name of the audio file to read samples from

## Dependencies

* libsndfile is used to open and decode input sound files
* The ConfigSystem module is required to set the name of the audio file to use

