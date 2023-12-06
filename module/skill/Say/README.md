# Say

## Description

Module that enables the robot to vocalize specified text using a text-to-speech tool and optionally perform a nodding gesture to indicate speaking.

## Usage

Include this module whenever the robot needs to vocalize or acknowledge messages via speech-like actions.

## Consumes

- `message::skill::Say`: A Task requesting to vocalize a text string. Contains the text to be spoken and an optional nod gesture request.

## Emits

- `message::actuation::HeadSequence`: A Task to instruct the robot's head to perform specific sequences, such as a nodding gesture.

## Dependencies

- **mimic3**: External python based command-line text-to-speech tool.
- **aplay**: Command-line sound player for Linux.
