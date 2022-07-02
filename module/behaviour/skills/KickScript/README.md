# Kick Script

## Description

Executes a scripted style kick based on a specified type (normal/penalty).

For a normal type kick, the kicking leg (left/right) can also be specified.

## Usage

Include this module to allow the robot to execute scripted kicks.

## Emits

- `extension::ExecuteScriptByName` to run scripted kick scripts
- `FinishKick` to signal kick to finish
- `message::motion::KickFinished` to signal that kick has finished
- `utility::behaviour::ActionPriorities` signals when the module's priority changes
- `utility::behaviour::RegisterAction` registers callbacks for executing or finishing kicks

## Dependencies

- The Script Engine module is required to execute the kick scripts
