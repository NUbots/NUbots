# GlobalConfig

## Description

Loads global robot identity configuration and publishes it as a message for other modules.

This module provides a single source of truth for:

- `player_id`
- `team_id`

Values are read from `GlobalConfig.yaml` and can be overridden by command-line arguments.

## Usage

Include this module early in your role so modules that depend on global robot identity can trigger correctly.

Optional command-line overrides:

- `--player_id <number>`
- `--team_id <number>`

If override values are invalid, the module logs an error and keeps the configured value.

## Consumes

- `extension::Configuration` from `GlobalConfig.yaml`
- `NUClear::message::CommandLineArguments`

## Emits

- `message::support::GlobalConfig`
