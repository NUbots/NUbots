# RobotCommunication

## Description

This module adds a communication schema that conforms to the official
robocup spec. It allows the robots to use this schema for internal communication.

The Robocup message is created, and emitted to other robots over UDP. All UDP messages are received by every robot in the network.

UDP messages that have been emitted by the receiving robot are filtered out.

The information that has been received over UDP is then emitted locally.

## Usage

Add this module to get information about other robots.

## Consumes

- `message::input::RoboCup` which contains data about the robot, including its state, current position, projected position and its view of the other robots.
- `message::behaviour::state::WalkState` which contains data about the robot's current movement.
- `message::input::GameState` which contains the current state of the game and penalised robots.
- `message::input::Sensors` which contains data from the robot's sensors, used to calculate its world position.
- `message::localisation::Ball` which contains information about the estimated position and velocity of the ball
- `message::localisation::Field` which contains information to convert values from world to field space
- `message::skill::Kick` which contains the direction and target of the robot's kick

## Emits

- `message::input::RoboCup` via UDP.
- `message::input::RoboCup` via local.

## Dependencies

- [Eigen Linear Algebra Library](https://eigen.tuxfamily.org/index.php)
