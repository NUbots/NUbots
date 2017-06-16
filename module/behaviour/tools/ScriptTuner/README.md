Script Tuner
============

## Description

Provides a curses interface for creating, editing and tweaking scripts.

## Usage

The program must be started with the name of a script file to edit as its sole
command line argument. If the file does not exist then a new blank script is
started and the file will be created when it is saved.

The keyboard controls are as follows:

* Up/Down arrow: select servo
* Left/right arrow: select angle or gain
* , (comma): go to previous frame
* . (period): go to next frame
* Enter: edit selected field
* Space: toggle motor lock
* S: save script
* T: edit frame duration
* N: insert a new frame before the current frame
* D: delete current frame

## Consumes

* `CommandLineArgumentis` containing the name of the script to edit
* `message::DarwinServos` containing the current positions of each servo

## Emits

* `message::ServoWaypoint>` to control servos

## Dependencies

* libncurses is used for the user interface
* The Darwin motion manager module is required to set waypoints

