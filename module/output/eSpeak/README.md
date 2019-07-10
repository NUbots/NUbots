eSpeak
======

## Description

Allows the robot to 'say' things using speech synthesis.

## Usage

This module uses the eSpeak speech synthesis engine to allow the robot to talk.
To make the robot speak, emit a `message::Say` containing the desired text. If
the robot is already talking it will wait until it has finished before starting
anything else.

The robot uses eSpeak's default voice.

## Consumes

* `message::Say` containing words for the robot to say

## Dependencies

* libespeak is used for speech synthesis

