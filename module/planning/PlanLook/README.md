# PlanLook

## Description

Plans how to look around. Follows a search pattern to find objects.

## Usage

Add this module to your role to make the robot look around when requested.

## Consumes

- `message::planning::LookAround` a Task requesting to run the search pattern to look around

## Emits

- `message::skill::Look` a Task requesting to look in a direction
- `message::planning::LookAround` a Task requesting to run the search pattern to look around the environment

## Dependencies
