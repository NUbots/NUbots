Getup
=============

## Description

Allows the robot to get up when it detects that it has fallen over.

## Usage

Include this module to allow the robot to get up after it has fallen over.

## Consumes

- `message::motion::ExecuteGetup` tells this module to call getup script
- `message::motion::KillGetup` disables getup and sets priority
- `message::platform::RawSensors` used to check if robot has fallen

## Emits

- `extension::ExecuteScriptByName` to run getup and stand scripts
- `message::motion::ExecuteGetup` instigates a getup
- `message::motion::KillGetup` stops a getup
- `utility::behaviour::ActionPriorities` signals when the module's priority changes
- `utility::behaviour::RegisterAction` registers callbacks for starting or stopping getups

## Dependencies

- The Script Engine module is required to execute the scripts
- The Filtered Sensors module is required to retrieve the current orientation
