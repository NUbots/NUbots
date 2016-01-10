Audio Input
===========

## Description

Records sound from a microphone or other available system audio input and
regularly emits it in chunks.

## Usage

AudioInput opens the default audio input device at its highest available sample
rate, 16-bit depth and however many channels are available. During this
initialisation it emits a `message::SoundChunkSettings` to notify other
modules of the format of upcoming sound chunks: the sample rate, number of
channels and number of samples that will be present in each chunk.

Once running, the module regularly emits buffers of audio samples in
`message::SoundChunk` objects. The number of chunks to emit per second is set
by the `CHUNKS_PER_SECOND` constant, which is
currently set to 100 (producing 100 10ms chunks per second).

Both AudioInput and the AudioFileInput module emit the same messages, so it is
possible to emulate microphone input from a pre-recorded file if necessary.

## Emits

* `message::SoundChunk` containing recorded audio samples
* `message::SoundChunkSettings` to describe the sample format of upcoming
  sound chunks

## Dependencies

* librtaudio is used to communicate with audio devices

