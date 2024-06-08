# Servos

## Description

Takes specific servo requests and emits a ServoGoal for them. Uses Director DSL such that each servo can only be accessed by one Provider at a time.
Individual Servo Providers are Done when their finish time is reached.
Limb Providers are Done when all of the Servos in the Limb are Done.
Sequence Providers run a sequence of Limbs one after the other. They are done when the final Limb is Done.

## Usage

Emit a servo, limb or sequence task, with priority to run according to the Director algorithm.

## Consumes

- Servo messages from `message/actuation/Servos.proto`.
- Limb messages from `message/actuation/Limbs.proto`, which will then emit Servo Tasks from `message/actuation/Servos.proto` to be consumed as above.
- Sequence messages from `message/actuation/Limbs.proto`, which will then emit Limb Tasks from `message/actuation/Limbs.proto` to be consumed as above.

## Emits

- `message::actuation::ServoGoal` the servo control information for the platform module.
- Servo messages from `message/actuation/Servos.proto` when a `message/actuation/Limbs.proto` is received.
- Limb messages from `message/actuation/Limbs.proto` when a Sequence message from `message/actuation/Limbs.proto` is received.

## Dependencies
