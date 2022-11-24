NSGA2Evaluator
==============

## Description

The evaluator half of the Multi_Objective Non-Dominated Sorting Genetic Algorithm II (NSGA2) optimisation of our walk engine. Evaluates walk engine parameters for an individual from the NSGA2Optimiser module for fitness, using the robot sway and robot travel distance or distance in radians of rotation.

## Usage

Thew NSGA2Optimiser module uses this module. Please see NSGA2Evaluator.yaml to set contants for boundry testing and scallers for multipath. No other human intervention required in this module.

## Consumes
- `Event` (Evaluator state - WAITING_FOR_REQUEST, SETTING_UP_TRIAL, RESETTING_SIMULATION, EVALUATING, TERMINATING_EARLY, TERMINATING_GRACEFULLY, and FINISHED)
- `WalkEvaluator` (Walk evaluation task)
- `StrafeEvaluator` (Strafe evaluation task)
- `RotationEvaluator` (Rotation evaluation task)
- `StandEvaluator` (Stand evaluation task)

## Emits

- `NSGA2FitnessScores` (to NSGA2Optimiser)
- `MotionCommand` (to WalkEngine)
- `ExecuteScript` (to ScriptEngine, to stand up, and stand still)

## Dependencies

- `NSGA2Terminate` (from NSGA2Optimiser, to finish the optimisation)
- `NSGA2EvaluationRequest` (from NSGA2Optimiser, to request evaluation of an individual)
- `Sensors` (actual name to be determined, has sensor data from the simulation, for calculating fitness)
