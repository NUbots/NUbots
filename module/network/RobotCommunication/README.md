# RobotCommunication

## Description

This module adds a communication schema that conforms to the official
robocup spec. It allows the robots to use this schema for internal communication.

The RoboCup message is created, and emitted to other robots over UDP. All UDP messages are received by every robot in the network.

UDP messages that have been emitted by the receiving robot are filtered out.

The information that has been received over UDP is then emitted locally.

Team messages are sent on port `10000 + team_number` with a maximum payload size of 512 bytes. Messages are counted only during
Ready, Set, and Playing game states toward the 12000 message budget per game.

When `share_detections` is on, the message also carries this robot's latest ball and robot detections (from
`localisation::CooperativeLocalisation`) in the NUbots team data field (`nubots`, tag 101 = 100 + our GameController team
number). Positions are whole centimetres in the same mixed-team field frame as `current_pose`, with the camera position
and the age of the camera frame, so teammates can fuse them into their cooperative ball and robot tracks. Detections
older than `shared_detection_max_age` are not sent, and at most `max_shared_balls` and `max_shared_robots` are sent, to
stay within the 128 byte team data budget.

## Usage

Add this module to get information about other robots.

## Consumes

- `message::input::RoboCup` which contains data about the robot, including its state, current position and projected position.
- `message::behaviour::state::WalkState` which contains data about the robot's current movement.
- `message::input::GameState` which contains the current state of the game and penalised robots.
- `message::input::Sensors` which contains data from the robot's sensors, used to calculate its world position.
- `message::localisation::Ball` which contains information about the estimated position and velocity of the ball
- `message::localisation::Field` which contains information to convert values from world to field space
- `message::skill::Kick` which contains the direction and target of the robot's kick
- `message::localisation::CooperativeDetections` which contains this robot's latest ball and robot detections on the field, to share with teammates

## Emits

- `message::input::RoboCup` via UDP.
- `message::input::RoboCup` via local.

## Dependencies

- [Eigen Linear Algebra Library](https://eigen.tuxfamily.org/index.php)
