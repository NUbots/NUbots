# NSGA2Evaluator

## Description

The evaluator half of the Multi-Objective Non-Dominated Sorting Genetic Algorithm II (NSGA2) optimisation of our walk engine. Evaluates walk engine parameters for an individual from the NSGA2Optimiser module for fitness, using the robot sway and robot travel distance or distance in radians of rotation.

## Usage

The NSGA2Optimiser module uses this module. Please see 'NSGA2Evaluator.yaml' to set constants for boundary testing and scalars for multipath. No other human intervention is required in this module.

## Consumes

- `module::support::optimisation::Event` Describes the current evaluator state. Possible values include WAITING_FOR_REQUEST, SETTING_UP_TRIAL, RESETTING_SIMULATION, EVALUATING, TERMINATING_EARLY, TERMINATING_GRACEFULLY, and FINISHED.
- `module::support::optimisation::WalkEvaluator` A walk evaluation task.
- `module::support::optimisation::StrafeEvaluator` A walk strafe evaluation task.
- `module::support::optimisation::RotationEvaluator` A walk rotation evaluation task.
- `module::support::optimisation::StandEvaluator` A stand evaluation task.
- `message::support::optimisation::NSGA2Terminate` Coordinates shutting down of the evaluator.
- `message::support::optimisation::NSGA2EvaluationRequest` Enables the evaluation of individuals.

## Emits

- `message::support::optimisation::NSGA2FitnessScores` Containing the score evaluation of an individual, to be used by the NSGA2Optimiser.

## Dependencies

- `message::platform::RawSensors` Contains sensor data from the simulation which is used for calculating an individual's fitness.
