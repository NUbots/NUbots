# NBSPlayback

## Description

## Usage

In order to play back the messages, you need to create a role with this module that has the modules you wish to test. You then need to enable the message types that are the input to these modules by setting them to true in the configuration (e.g. `message.input.sensors: true`). Make sure the `.nbs` file contains the messages you are trying to emit (you can check with `./b nbs stats file`).

Provide `.nbs` file paths via the command line. For example `./b run <role> recordings/<nbs-file-a> recordings/<nbs-file-b>`

## Emits

- `message.nbs.PlaybackNBS`: The message containing NBS file paths, message types to play and playback mode.

## Dependencies

Modules:

- nbs/Player
