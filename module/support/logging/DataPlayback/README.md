# DataPlayback

## Description

DataPlayback is a module which will allow you to play back `.nbs` files as if they were happening in realtime.
This can be very useful for when you are trying to diagnose a problem for which you have a recording as you can continuously play back the same data.

The module will buffer up packets from the future and emit them at the same rate that they were emitted originally.

## Usage

In order to play back the messages, you need to create a role with this module that has the modules you wish to test. You then need to enable the message types that are the input to these modules by setting them to true in the configuration (e.g. `message.input.Sensors: true`). Make sure the `.nbs` file contains the messages you are trying to emit (you can check with `./b nbs stats file`).

There are two methods to provide nbs files to this module, via the configuration file or via the command line. If the configuration file has arguments those are used first. If there are no files provided in the configuration file, the arguments from the command line are used instead. This is so that if you have another module in your role that relies on command line arguments you have an alternate way to provide these files.

You can provide multiple `.nbs` files and they will be played in sequence. For example `./b run <role> recordings/<nbs-file-a> recordings/<nbs-file-b>` will run the role and play back

### Notes

There is some weird behaviour that will occur when using this module that won't happen in a real system. Take note of these if they influence your usage

- If there are any timestamp fields in the messages, these will not be updated. If you are comparing the timestamps in the messages to the current time they will be wrong.
- When you are providing paths to `.nbs` files via a `./b run` command they are relative to the build directory in docker. There is a symlink to `recordings` that is placed there to make any nbs file in recordings easier to use but outside of this path you will need to provide the full path from dockers perspective.
