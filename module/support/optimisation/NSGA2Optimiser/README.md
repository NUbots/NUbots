NSGA2Optimiser
==============

## Description
The optimiser half of the Multi-Objective, Non-Dominated Sorting Genetic Algorithm II (NSGA2) optimisation of our walk engine. Optimises the walk parameters using an NSGA2 algorithm, using the NSGA2Evaluator module to run the evaluations. The NSGA2 library [here](./src/nsga2/) was adapted from https://github.com/dojeda/nsga2-cpp.
There are individual optimisation tasks. These are contained in the tasks folder. (The current tasks available are - Walk, Strafe, Rotate, and a Multipath). The parameters to be optimised can be found in the NSGA2Optimiser.yaml file.

## Usage
As mentioned above the NSGA2.yaml file contains the parameters for running this module. The search space width (population size) and depth (number of generations), the number of objectives to be evaluated and simulation time limit are set here. The simulated binary crossover and mutation values are chosen here as well.
The task must be specfied in this file. Eg./ task: "walk"

## Consumes
- `message::platform::webots::WebotsReady` Starts the first evaluation once Webots is ready.
- `message::support::optimisation::NSGA2FitnessScores` Containing the score evaluation of an individual, received from NSGA2Evaluator.
- `module::support::optimisation::WalkOptimiser` A walk optimisation task.
- `module::support::optimisation::StrafeOptimiser` A walk strafe optimisation task.
- `module::support::optimisation::RotationOptimiser` A walk rotation optimisation task.
- `module::support::optimisation::StandOptimiser` A stand optimisation task.
- `module::support::optimisation::MultiPathOptimiser` A multipath walk optimisation task.

## Emits
- `message::platform::webots::WebotsReady` (to Webots. On startup ready)
- `message::support::optimisation::NSGA2EvaluatorReady` Emitted when initialisation has been completed to signal that evaluation can begin.
- `message::support::optimisation::NSGA2Terminate` Sent to NSGA2Evaluator to stop the optimisation and shut down the powerplant.
- `message::support::optimisation::NSGA2EvaluationRequest` Sent to NSGA2Evaluator to trigger the evaluation of a new individual.(to NSGA2Evaluator, to request evaluation of an individual)
- `task->make_evaluation_request` (to NSGA2Evaluator.  Request an evaluation of an individual)

## Dependencies
- `message::support::optimisation::NSGA2FitnessScores` (from NSGA2Evaluator)
