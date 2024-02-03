# MessageLogHandler

## Description

This module provides an interface to take the LogMessage and ReactionStatistics messages from NUClear and recodes them into protobuf messages.
These messages can then be sent using NetworkForwarder or recorded using DataLogging.
Additionally it can be configured to ignore the display level of the logs so that the level can be adjusted at a later time for analysis.

## Usage

Include this module and configure NetworkForwarder or DataLogging to forward the LogMessage and ReactionStatistics messages.

## Consumes

- NUClear::message::LogMessage
- NUClear::message::ReactionStatistics

## Emits

- message::nuclear::LogMessage
- message::nuclear::ReactionStatistics

## Dependencies

None
