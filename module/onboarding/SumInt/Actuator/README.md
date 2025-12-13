# Actuator

## Description

Module that takes in the final sum result and makes the robot nod.

## Usage

Include it in your role as well as the other two modules (Summer and Comparator).

## Consumes

- `message::onboarding::SumInt::FinalResult` containing the total result of the summing.

## Emits

- `message::actuation::BodySequence` containing the servo instructions read from the NodYes.yaml script needed to nod.

## Dependencies

- `actuation::Servos` to process the NodYes command.
