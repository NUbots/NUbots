# OnboardWalkOptimisation

## Description

This module manages the use of running an optimisation routine onboard the real robot platform. It handles the period where the robot resets after an optimisation trial run and it checks for stability.

## Usage

Use this with an optimisation module, such as the NSGA2Evaluator and NSGA2Optimiser, to run optimisation onboard the real robot platform.

## Consumes

- `message::support::optimisation::OptimisationCommand` containing information on if the optimisation is to be reset or terminated.
- `message::input::Sensors` to determine if the robot is stable.

## Emits

- `message::support::optimisation::OptimisationResetDone` to tell the optimisation when the robot has been reset for a new trial run.
- `message::support::optimisation::OptimisationRobotPosition` to tell the optimiser where the robot is in the world.

## Dependencies

- An optimiser
