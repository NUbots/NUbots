# Lyrics2Logs

## Description

Reads lyrics from a file and logs them one line at a time at the time they are played. Used for testing/demonstrating the logs feature in NUsight.

## Usage

- Enable `message.nuclear.LogMessage`, and `message.nuclear.ReactionStatistics` in `NetworkForwarder.yaml` to send to NUsight and/or in `DataLogging.yaml` to save logs to an NBS file.
- Build and the `lyrics2logs` role
- Run the role and run NUsight

## Consumes

None

## Emits

- `message::support::logging::LyricsPlayTime``

## Dependencies

None
