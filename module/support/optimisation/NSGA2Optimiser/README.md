# NSGA2Optimiser

## Description

The optimiser half of the Multi-Objective, Non-Dominated Sorting Genetic Algorithm II (NSGA2) optimisation algorithm. Optimises parameters using an NSGA2 algorithm, using the NSGA2Evaluator module to run the evaluations.

The NSGA2 library [here](./src/nsga2/) was adapted from https://github.com/dojeda/nsga2-cpp.

There are individual optimisation tasks. These are contained in the tasks folder.
The current tasks available are:

- Walk forwards
- Strafe walk
- Rotate walk
- Multipath walk

The parameters to be optimised can be found in the 'NSGA2Optimiser.yaml' file.

## Usage

The NSGA2.yaml file contains the parameters for running this module. The parameters to set are:

- Search space width (population size)
- Search space depth (number of generations)
- Number of objectives to be evaluated
- Trial time limit
- Simulated binary crossover value
- Mutation value
- The task, e.g. "walk"

## Consumes

- `message::support::optimisation::NSGA2FitnessScores` Containing the score evaluation of an individual, received from NSGA2Evaluator.
- `module::support::optimisation::WalkOptimiser` A walk optimisation task.
- `module::support::optimisation::StrafeOptimiser` A walk strafe optimisation task.
- `module::support::optimisation::RotationOptimiser` A walk rotation optimisation task.
- `module::support::optimisation::StandOptimiser` A stand optimisation task.
- `module::support::optimisation::MultiPathOptimiser` A multipath walk optimisation task.

## Emits

- `message::support::optimisation::NSGA2EvaluatorReady` Emitted when initialisation has been completed to signal that evaluation can begin.
- `message::support::optimisation::NSGA2Terminate` Sent to NSGA2Evaluator to stop the optimisation and shut down the powerplant.
- `message::support::optimisation::NSGA2EvaluationRequest` Sent to NSGA2Evaluator to trigger the evaluation of a new individual.
- `task->make_evaluation_request` (to NSGA2Evaluator. Request an evaluation of an individual)

## Dependencies

- `message::support::optimisation::NSGA2Evaluator` Runs evaluations of individuals that are used to optimise parameters.
