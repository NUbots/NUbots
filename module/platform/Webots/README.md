webots
======

## Description
A module to connect to Webots controllers and exchange protobuf messages with them.

## Usage
Add the ip for the computer running Webots, and the port the controller is listening on. For Robocup the port determines
which player this robot is.

You'll need to build this codebase (NUbots), and build the controller you're running in the NUWebots repository.

Start the world in webots, then do `./b run webots`, `./b run webots_keyboard`, or another associated role from
this repositiory. If you disconnect or need to restart the world, stop the role running with `ctrl + c`, then refresh
the world in webots and run the role with the same `./b` command as above.

## Emits
platform::RawSensors
output::CompressedImage

## Dependencies
Configuration
