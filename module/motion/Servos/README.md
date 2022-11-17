# Servos

## Description

Takes specific servo requests and emits a ServoTarget for them. Uses Director DSL such that each servo can only be accessed by one Provider at a time.
Individual servo Providers are Done when their finish time is reached.
Limb Providers are Done when all of the servos in the limb are Done.

## Usage

Emit a servo or limb task, with priority to run according to the Director algorithm.

## Consumes

- Servo messages from `message/motion/Servos.proto`.
- Limb messages from `message/motion/Limbs.proto`, which will then emit Tasks from `message/motion/Servos.proto` to be consumed as above.

## Emits

- `message::motion::ServoTarget` the servo control information for the platform module
- Servo messages from `message/motion/Servos.proto` when a `message/motion/Limbs.proto` is received.

## Dependencies
