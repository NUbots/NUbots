NSGA2Optimiser
==============

## Description

The optimiser half of the Non-Dominated Sorting Genetic Algorithm II (NSGA2) optimisation of our walk engine. Optimises the walk parameters using an NSGA2 algorithm, using the NSGA2Evaluator module to run the evaluations. The NSGA2 library [here](./src/nsga2/) was adapted from https://github.com/dojeda/nsga2-cpp.

## Usage


## Emits

- `NSGA2Terminate` (to NSGA2Evaluator, to stop the optimisation)
- `NSGA2EvaluationRequest` (to NSGA2Evaluator, to request evaluation of an individual)

## Dependencies

- `NSGA2FitnessScores` (from NSGA2Evaluator)
