# SoccerConfig

## Description

Loads soccer field dimensions from configuration and publishes a `FieldDescription` message used by localisation,
vision, and behaviour modules.

In addition to copying field constants from YAML, the module computes derived values such as goalpost positions and
goalpost top height.

## Usage

Include this module in roles that require field geometry.

The module listens to configuration updates for `FieldDescription.yaml` and re-emits an updated
`message::support::FieldDescription` when the file changes.

Configuration files are located at:

- `module/support/configuration/SoccerConfig/data/config/FieldDescription.yaml`
- `module/support/configuration/SoccerConfig/data/config/webots/FieldDescription.yaml`

Key configured values include field size, line width, goal dimensions, penalty area dimensions, and ball radius.

## Consumes

- `extension::Configuration` from `FieldDescription.yaml`

## Emits

- `message::support::FieldDescription`
