# FieldLocalisationNLopt

## Description

A localisation method for estimating the where the field is in world space, which relies on field line points and
field line intersections.

## Usage

Include this module to allow the robot to estimate where the field is in world space.

## Consumes

- `message::vision::FieldLines` uses the field line observations from FieldLineDetector module
- `message::vision::FieldLineIntersections` uses the field line intersections from FieldLineDetector module

## Emits

- `message::localisation::Field` contains the estimated (x, y, theta) state

## Dependencies

- `Eigen`
- `utility::math::stats::MultivariateNormal` Utility for sampling from a multivariate normal distribution
