# Ready

## Description

This module contains a Provider for the Ready Task, which aims to position the robots on the field in the RoboCup READY phase. The robot moves for a configured amount of time with a configured walk command and then stands still

## Usage

Add this module to walk onto the field in the READY state.

## Consumes

- `message::strategy::Ready` a Task requesting to walk to the ready position

## Emits

- `message::skill::Walk` a Task requesting to walk at a particular speed
- `message::strategy::StandStill` a Task requesting to stand still

## Dependencies

- The walk engine
