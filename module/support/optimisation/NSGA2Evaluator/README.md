# NSGA2Evaluator

## Description

The evaluator half of the Multi-Objective Non-Dominated Sorting Genetic Algorithm II (NSGA2) optimisation algorithm. Evaluates parameters for an individual from the NSGA2Optimiser module based on fitness.

Currently contains implementations for optimising the walk engine.

## Usage

The NSGA2Optimiser module uses this module. Please see 'NSGA2Evaluator.yaml' to set constants for boundary testing and scalars for multipath.

To implement the evaluator for a new optimisation scenario, extend from the EvaluatorTask class.

## Consumes

- `module::support::optimisation::Event` Describes the current evaluator state. Possible values include WAITING_FOR_REQUEST, SETTING_UP_TRIAL, RESETTING_TRIAL, EVALUATING, TERMINATING_EARLY, TERMINATING_GRACEFULLY, and FINISHED.
- `module::support::optimisation::WalkEvaluator` A walk evaluation task.
- `module::support::optimisation::StrafeEvaluator` A walk strafe evaluation task.
- `module::support::optimisation::RotationEvaluator` A walk rotation evaluation task.
- `module::support::optimisation::StandEvaluator` A stand evaluation task.
- `message::support::optimisation::NSGA2Terminate` Coordinates shutting down of the evaluator.
- `message::support::optimisation::NSGA2EvaluationRequest` Enables the evaluation of individuals.
- `message::input::Sensors` Contains sensor data which is used for calculating an individual's fitness.

## Emits

- `message::support::optimisation::NSGA2FitnessScores` Containing the score evaluation of an individual, to be used by the NSGA2Optimiser.

## Dependencies

- `module::support::optimisation::NSGA2Optimiser` Optimises parameters based on the performance of individuals that are evaluated.
