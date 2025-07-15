# Player

## Description

The Player module enables playback of `.nbs` files with support for different playback modes and speeds. It manages the timing and emission of messages from NBS files while maintaining accurate timing through NUClear clock adjustments.

## Features

- Multiple playback modes:
  - FAST: Skips idle time between messages while maintaining relative timing
  - REALTIME: Plays messages at their original timing
  - SEQUENTIAL: Processes messages one at a time with manual advancement
- Adjustable playback speed (2^n multiplier)
- Support for loading multiple NBS files
- Message type filtering
- Playback state monitoring

## Consumes

- `message.eye.ScrubberSetModeRequest`: Set the playback mode (FAST, REALTIME, or SEQUENTIAL)
- `message.eye.ScrubberLoadRequest`: Load NBS file(s) and specify which message types to enable
- `message.eye.ScrubberPlayRequest`: Start playback in the current mode
- `message.eye.ScrubberPauseRequest`: Pause the current playback
- `message.eye.ScrubberSetPlaybackSpeedRequest`: Adjust the playback speed

### Emits

- The module can emit any message type that is found in the `.nbs` files and enabled in the configuration.
  Each message is emitted at the appropriate time according to the selected playback mode.

- `message.eye.ScrubberPlaybackFinished`: Emitted when playback reaches the end of the NBS file(s)
- `message.eye.ScrubberState`: Contains current playback information:
  - Current message number
  - Total message count
  - Current timestamp
  - Start and end timestamps
  - Playback state (PLAYING, PAUSED, ENDED)
  - Current playback speed
