# FileLogHandler

## Description

Writes NUClear log output and reaction exceptions to a log file on disk.

The module listens to runtime log messages and reaction statistics, formats them with source information and level, and
appends them to the configured file.

If a reaction reports an exception, it records exception details (and stack information in debug builds).

## Usage

Configure the output file path in `FileLogHandler.yaml`:

- `log_file`: absolute or relative path to the log file

Behavior:

- Opens the configured file in append mode
- Adds a separator line when configuration is (re)loaded
- Serialises writes with a mutex
- Closes the file on shutdown

## Consumes

- `extension::Configuration` from `FileLogHandler.yaml`
- `NUClear::message::LogMessage`
- `NUClear::message::ReactionStatistics`
- `NUClear::Shutdown`
