# Player

## Description

The Playback module enables fast playback of `.nbs` files by emitting messages and skipping idle times between
them. It adjusts the NUClear clock to ensure the timing remains accurate.

## Usage

Emit `message.nbs.StartPlayback` elsewhere in system to playback NBS file

## Emits

- `message.nbs.FinishedPlayback` Emitted when playback is finished
