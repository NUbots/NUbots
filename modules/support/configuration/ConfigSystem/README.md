Configuration System
====================

## Description

This module loads JSON-formatted configuration files and emits the data within
to the modules that have requested it. Additionally it watches the configuration
directory and automatically reloads files when they are modified.

## Usage

Any Reactor that wishes to use the config system should set a trigger on
`messages::Configuration<Type>`, where `Type` is a class with a public static
member called `CONFIGURATION_PATH`, set to the name of the configuration file
or directory to load. All paths are relative to the base configuration
directory, which is 'config/'.

Config files must have a .json extension, all other files are ignored by the
directory watching thread. If a directory is specified as the config path then
all JSON files directly inside it will be loaded and it will be watched for new
or changed files. Subdirectories of these directories are not loaded
recursively.

ConfigSystem will emit a `Configuration` each time it loads a file, i.e. once
for each config file at initialisation and again every time a file is modified.
The `name` member is set to the filename of the JSON file (not including any
path) and `configuration` member is a `messages::ConfigurationNode` that can be
accessed similarly to a map. For example, given a JSON file `{ "foo": "bar" }`,
`configuration["foo"]` would equal `"bar"`.

## Emits

* `messages::Configuration<Type>` containing the configuration data for the
  reactor `Type`.

## Dependencies

* The JSON parser uses the jsmn library for lexing.
* The directory watching code uses Linux-specific system calls (inotify).

