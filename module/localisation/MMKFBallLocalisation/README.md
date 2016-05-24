Localisation
============

## Description

This is a multi-noise-model kalman filter for localising a ball. It uses multiple process noise update models to model the ball being kicked as well as at rest, emitting the best hypoethesis at any time. The model also includes functionality to switch between cartesian and spherical observation updates, which help observation updates close to the robot. Odometry updates are included indirectly by modifying the state and adding the relevant noise factors (think of this as linearising the kalman filter for odometry only).

## Usage


## Emits

- Ball Localisation Objects

## Dependencies

- Ball Vision Objects
- Odometry
