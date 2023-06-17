# FieldLocalisation

## Description

A particle filter based localisation method for estimating the where the field is in world space, which relies solely on
field line observations.

## Usage

Include this module to allow the robot to estimate where the field is in world space.

## Consumes

- `message::vision::FieldLines` uses the field line observations from FieldLineDetector module

## Emits

- `message::localisation::Field` contains the estimated (x, y, theta) state and covariance

## Dependencies

- `Eigen`
- `utility::math::stats::MultivariateNormal` Utility for sampling from a multivariate normal distribution
