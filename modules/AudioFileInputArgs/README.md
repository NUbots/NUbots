Audio File Input with Arguments
===============================

## Description

Reads sound from a file specified as a command line argument and emits it in
regular-sized chunks as if it were being recorded in real time.

## Usage

AudioFileInputArgs opens the audio file specified in the last command-line
argument to the exacutable. It then emits a `messages::SoundChunkSettings`
containing the sound file's sample rate, bit depth and the number of samples
that will be in each sound chunk. It then emits a `messages::SoundFileStart`
containing the filename and time at start pf playback.

Once running, the module regularly emits buffers of audio samples in
`messages::SoundChunk` objects. The number of chunks to emit per second is set
by the `CHUNKS_PER_SECOND` constant, which is currently set to 100 (producing
100 10ms chunks per second).

Sound chunks are created in real time - five minutes worth of audio in the
source file will be emitted in chunks over a five minute period.

This module and AudioInput both emit audio data using the same messages, as
such it is possible to use AudioFileInput to effectively emulate microphone
input with pre-recorded sound.

## Consumes

* `NUClear::CommandLineArguments` to retrieve a file name from the command-
  line arguments.

## Emits

* `messages::SoundChunk` containing audio samples read from the file
* `messages::SoundChunkSettings` to describe the sample format of upcoming
  sound chunks
* `messages::SoundFileStart` to indicate the beginning of a sound file

## Dependencies

* libsndfile is used to open and decode input sound files

