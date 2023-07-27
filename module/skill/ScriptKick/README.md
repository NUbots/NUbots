# ScriptKick

## Description

Executes a script for kicking.

## Usage

Emit a Kick message when wanting to kick.

## Consumes

`message::skill::Kick` the kick information to use when kicking.

## Emits

`message::actuation::LimbsSequence` through the Script extension, which populates the requested Script/s into LimbsSequences.

## Dependencies

- Script extension
