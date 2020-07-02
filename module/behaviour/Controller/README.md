Behaviour Controller
=============

## Description

Controls which of the actions a robot is capable of should run.

## Usage

This module is used to control which of a range of actions are allowed to execute.

## Consumes

* `message::behaviour::RegisterAction` containing the details and motor requests of the action
* `message::behaviour::ActionPriority` updates the priority of an action
* 
## Emits

* `message::motion::ServoWaypoint` which instructs the motion manager to run.
* 
## Dependencies

* The motion manager is required to pass the control messages to cause the changes
