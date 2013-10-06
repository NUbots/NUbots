Darwin Motion Manager
=====================

## Description

Manages movement of the Darwin by providing timed waypoints for each servo.

## Usage

To move the robot, it is usually necessary to perform many servo adjustments,
with specific timing and in a specific order. This module represents these
adjustments as waypoints - instructions that a servo should go to a specified
position at a given time using given gain. A queue of waypoints is maintained
for each of the Darwin's servos.

Servos are updated at a rate of 50Hz, using the most recent sensor data. A
`messages::ServoWaypointsCompleted<messages::DarwinSensors::Servo::ID>` is
emitted whenever all waypoints on a servo's queue have been completed (with
the template parameter being the relevant servo's ID). When all queues have
been completed a `messages::AllServoWaypointsComplete` is also emitted.

To set a waypoint, emit a `messages::ServoWaypoint` containing the servo ID,
desired position, gain and the time by which the movement should finish. Any
other waypoints already in that servo's queue that would have started after the
new waypoint are deleted. Multiple waypoints can be set at once by emitting
them in a `std::vector`.

## Consumes

* `messages::DarwinSensors` to find the current status of the Darwin's servos
* `messages::ServoWaypoint` to set a single waypoint for a servo to reach
* `std::vector<messages::ServoWaypoint>` to set multiple waypoints at a time

## Emits

* `messages::ServoWaypointsComplete<messages::DarwinSensors::Servo::ID>` when
  all waypoints for a specific servo have been reached
* `messages::AllServoWaypointsComplete` when all waypoints for all servos have
  been reached

## Dependencies

* The Darwin Hardware I/O module is required to communicate with the robot

