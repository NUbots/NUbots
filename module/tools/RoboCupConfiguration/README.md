# RoboCupConfiguration

## Description

IMPORTANT: `systemconfiguration` and `robocup` roles should be built to the robot so that the relevant configuration files exist, if they do not already exist.

The purpose of this module is to provide a simple way to set up the robot for RoboCup games. It has an ncurses inferface that shows the current configuration of the network and certain module configuration files. The user can modify these fields to set up the robot appropriately. Note that this is designed to be run directly on the robot. To revert the affects of this module, rebuild configuration files to the robot and remove

- `system/nugus<number>/etc/systemd/network/30-wifi.network`
- `system/nugus<number>/etc/wpa_supplicant/wpa_supplicant-<wifi_interface>.conf`

Then run `./systemconfiguration` and reboot. Alternatively you can run this module again and revert the values manually.

## Usage

This module is to be used right before a RoboCup game to quickly set up the robot with the game-specific details.

The interface has the following commands (shift-letter for uppercase)

- Arrow keys: to move around the display.
- Enter: to edit the selected field with user input.
- Space: to toggle the value of the selected field.
- R: reads the current config and sets the display, overwritting non-applied user inputs.
- C: uses the current values shown in the display to write to the relevant configuration files.
- N: applies network changes specified, restarts network services
- S: starts the robocup service, which will start the robocup binary, and will rerun if it stops
- D: stops the robocup service
- E: enables wifi, useful for if it has been disabled and the robot is about to play a game.
- W: disables wifi, useful for after a RoboCup game as wifi congestion is an issue at RoboCup and it is important to disconnect from any field networks when not playing.
- X: shutdown the program

## Consumes

N/A

## Emits

N/A

## Dependencies

N/A
