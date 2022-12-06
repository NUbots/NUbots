# Servos

## Description

Takes specific servo requests and emits a ServoTarget for them. Uses Director DSL such that each servo can only be accessed by one Provider at a time.
Individual Servo Providers are Done when their finish time is reached.
Limb Providers are Done when all of the Servos in the Limb are Done.
Sequence Providers run a sequence of Limbs one after the other. They are done when the final Limb is Done.

## Usage

Emit a servo, limb or sequence task, with priority to run according to the Director algorithm.

## Consumes

- Servo messages from `message/motion/Servos.proto`.
- Limb messages from `message/motion/Limbs.proto`, which will then emit Servo Tasks from `message/motion/Servos.proto` to be consumed as above.
- Sequence messages from `message/motion/Limbs.proto`, which will then emit Limb Tasks from `message/motion/Limbs.proto` to be consumed as above.

## Emits

- `message::motion::ServoTarget` the servo control information for the platform module.
- Servo messages from `message/motion/Servos.proto` when a `message/motion/Limbs.proto` is received.
- Limb messages from `message/motion/Limbs.proto` when a Sequence message from `message/motion/Limbs.proto` is received.

## Dependencies
