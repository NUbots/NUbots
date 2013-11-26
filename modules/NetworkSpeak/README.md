NetworkSpeak
======

## Description

Allows the robot to 'say' things using speech synthesis by sending it packets over the network.

## Usage

This module uses the eSpeak speech synthesis engine to allow the robot to talk.
To make the robot speak, network emit a `messages::NetSay` containing the desired text. If
the robot is already talking it will wait until it has finished before starting
anything else.

The robot uses eSpeak's default voice.

## Consumes

* `messages::NetSay` containing words for the robot to say

## Dependencies

* eSpeak module for actual speech synthesis.

