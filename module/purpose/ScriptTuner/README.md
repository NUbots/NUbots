# Script Tuner

## Description

Provides a curses interface for creating, editing and tweaking scripts.

## Usage

The program must be started with the name of a script file to edit as its sole
command line argument. If the file does not exist then a new blank script is
started and the file will be created when it is saved.

The keyboard controls are as follows:

- Up/Down arrow: Select servo
- Left/right arrow: Select angle or gain
- , (comma): Go to previous frame
- . (period): Go to next frame
- Enter: Edit selected field
- Space: Toggle motor lock
- : (colon): List available commands
- A: Save script as
- G: Allows multiple gain edits at once
- I: Delete current frame
- J: "jump" to frame without the robot moving
- M: Mirror script - flip about sagittal plane
- N: Insert a new frame before the current frame
- R: Refresh the view
- S: Save script
- T: edit frame duration
- X: Shutdown powerplant/stop binary

## Consumes

- `NUClear::message::CommandLineArguments` containing the name of the script to edit
- `module::behaviour::tools::LockServo` (internal to this class only) to trigger servo lock event
- `message::platform::RawSensors` to get position of specific servo when locking it

## Emits

- `message::actuation::ServoGoal` to control currently selected servo when locked or unlocked
- `message::actuation::ServoGoals` to control all servos when transitioning between frames
- `message::actuation::LimbsSequence` to play the script
- `module::behaviour::tools::LockServo` (internal to this class only) to trigger servo lock event

## Dependencies

- libncurses is used for the user interface
