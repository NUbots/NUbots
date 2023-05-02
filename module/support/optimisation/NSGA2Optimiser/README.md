NSGA2Optimiser
==============

## Description

The optimiser half of the Multi0Objective, Non-Dominated Sorting Genetic Algorithm II (NSGA2) optimisation of our walk engine. Optimises the walk parameters using an NSGA2 algorithm, using the NSGA2Evaluator module to run the evaluations. The NSGA2 library [here](./src/nsga2/) was adapted from https://github.com/dojeda/nsga2-cpp.
There are induvidual opti,isation tasks. These are contained in the Tasks folder. (Task are at the moment - Walk, Strafe, Rotate, and a Multipath). The parameters to be optimnised can be found in the NSGA2Optimiser.yaml file.

## Usage

As mentioned above the NSGA2.yaml file contains the parameters for ruunning this module. The search space width (population size) and depth (number of generations), the number of objectives to be evaluated and simulation time limit are set here. The simulated binary crossover and mutation values mutation are chosen here as well.
The task must be specfied in this file. Eg./ task: "walk"

## Consumes

- `WebotsReady` (Webots ready, starting first evaluation)
- `NSGA2FitnessScores` (Get evaluation fitness scores)
- `WalkOptimiser` (Task type is walk)
- `StrafeOptimiser` (Task type strafe)
- `RotationOptimiser` (Task type rotation)
- `StandOptimiser` (Task type stand)
- `MultiPathOptimiser` (Task type multipath")
-
## Emits

- `WebotsReady`(to Webots. On startup ready)
- `NSGA2EvaluatorReady`(Initialisation succeeded, evaluate the first individual of the first generation)
- `NSGA2Terminate` (to NSGA2Evaluator, to stop the optimisation. Shut down powerplant)
- `NSGA2EvaluationRequest` (to NSGA2Evaluator, to request evaluation of an individual)
- `task->MakeEvaluationRequest` (to NSGA2Elvaluator.  Request an evaluation of an individual)

## Dependencies

- `NSGA2FitnessScores` (from NSGA2Evaluator)
