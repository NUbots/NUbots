# ReactionTimer

## Description

Logs execution duration for reactions using NUClear reaction statistics.

For each `ReactionStatistics` message, it computes elapsed time from the reaction start and finish timestamps and logs
the result in milliseconds.

## Usage

Include this module when profiling or debugging reaction performance.

Configuration:

- `module/support/ReactionTimer/config/ReactionTimer.yaml`
- `log_level`

## Consumes

- `extension::Configuration` from `ReactionTimer.yaml`
- `NUClear::message::ReactionStatistics`
