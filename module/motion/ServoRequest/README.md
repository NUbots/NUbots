# ServoRequest

## Description

Takes specific servo requests and emits a ServoTarget for them. Uses Director DSL such that each servo can only be accessed by one Provider at a time.

## Usage

Emit a servo task, with priority to run according to the Director algorithm.

## Consumes

- Any servo message from `message/motion/Servos.proto`.
- Any limbs from `message/motion/Limbs.proto`, which will then emit any from `message/motion/Servos.proto` with `SCOPE::DIRECT` to be consumed as above.

## Emits

- `message::motion::ServoTarget` the servo control information for the platform module
- Servo messages from `message/motion/Servos.proto` when a `message/motion/Limbs.proto` is received, with with `SCOPE::DIRECT`.

## Dependencies
