NSGA2Evaluator
==============

## Description

The evaluator half of the Non-Dominated Sorting Genetic Algorithm II (NSGA2) optimisation of our walk engine. Evaluates walk engine parameters for an individual from the NSGA2Optimiser module for fitness, using the robot sway and ball travel distance.

## Usage


## Emits

- `NSGA2FitnessScores` (to NSGA2Optimiser)
- `MotionCommand` (to WalkEngine)
- `ExecuteScript` (to ScriptEngine, to stand up, and stand still)
- `GazeboWorldCtrl` (to be changed to webots, for resetting the simulation)

## Dependencies

- `NSGA2Terminate` (from NSGA2Optimiser, to finish the optimisation)
- `NSGA2EvaluationRequest` (from NSGA2Optimiser, to request evaluation of an individual)
- `Sensors` (actual name to be determined, has sensor data from the simulation, for calculating fitness)
- `GazeboWorldStatus` (actual name to be determined, has time data from the simulation)
- `GazeboBallLocation` (actual name to be determined)
- `GazeboRobotLocation` (actual name to be determined)
