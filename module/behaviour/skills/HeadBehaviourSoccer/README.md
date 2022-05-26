HeadBehaviourSoccer
=============

## Description

Controls the state of the head

## Usage

Include this module to allow the robot to focus on a ball or search for a ball if it is not visible

## Consumes

* `message::vision::Balls` uses vision ball to calculate angles to rotate head

## Emits

* `message::motion::HeadCommand` instructs the head to move to desired angle

## Dependencies
