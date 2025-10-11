# NBSPlayback

The NBSPlayback module enables replaying of `.nbs` (NUClear Binary Stream) files in various modes.
It emits recorded messages from the files and controls the NUClear clock to ensure timing accuracy based on the selected playback mode.
The module can dynamically emit any message type found in the `.nbs` files based on configuration settings.

## Usage

Provide `.nbs` file paths via the command line when running your role.
For example: `./b run <role> recordings/<nbs-file-a> recordings/<nbs-file-b>`

## Configuration

The module can be configured through `NBSPlayback.yaml`:

- `log_level`: Set the logging verbosity level
- `playback_mode`: Set the playback mode (FAST, REALTIME, or SEQUENTIAL)
- `progress_bar_mode`: Choose how to display progress:
  - `COUNT`: Show progress based on message count
  - `TIME`: Show progress based on elapsed time
- `messages`: Configure which message types to play back:
  ```yaml
  messages:
    message_type_1: true # Enable this message type
    message_type_2: false # Disable this message type
  ```

## Playback Modes

The module supports three playback modes:

- `FAST`: Emits messages as quickly as possible, skipping idle times
- `REALTIME`: Emits messages at the same rate they were recorded
- `SEQUENTIAL`: Emits one message at a time, waiting for all triggered tasks to complete before continuing

## Consumes

- `NUClear::message::CommandLineArguments` Path to the `.nbs` files to play from the command line.
- `message.eye.ScrubberState` Current state of the playback
- `message.eye.ScrubberPlaybackFinished` Notification when playback completes

## Emits

- `message.eye.ScrubberLoadRequest` Request to load NBS files
- `message.eye.ScrubberPlayRequest` Request to start playback
- `message.eye.ScrubberSetModeRequest` Request to set the playback mode

## Notes

- The module automatically starts playback when launched
- Progress is displayed in the terminal using a progress bar
- Invalid progress bar modes will be logged as errors
- The module will automatically close the progress bar when playback finishes
