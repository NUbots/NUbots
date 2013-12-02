/*
 * This file is part of Darwin Motion Manager.
 *
 * Darwin Motion Manager is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Darwin Motion Manager is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Darwin Motion Manager.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#include "messages/motion/ServoWaypoint.h"

#include "MotionManager.h"
#include "utility/math/angle.h"

namespace modules {
namespace platform {
namespace darwin {

    MotionManager::MotionManager(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Trigger<Every<20, std::chrono::milliseconds>>, With<messages::platform::darwin::DarwinSensors>>([this](const time_t& time, const messages::platform::darwin::DarwinSensors& sensors) {

                std::lock_guard<std::mutex> waypointLock(waypointMutex);

            bool emptiedQueue = false;

            // Firstly see if there are any old motions that have expired and remove them
            for (size_t i = 0; i < 20; ++i) {
                auto& queue = waypoints[i];

                while(!queue.empty() && queue.front().executed && queue.front().end < time) {
                    queue.pop_front();

                    // If we just totally emptied this event queue, emit an event that we finished this motor
                    if(queue.empty()) {
                        // Emit that we finished this queue
                        emptiedQueue = true;
                        queueEnd(i);
                    }
                }
            }

            // If we emptied a queue, check if they are all empty
            if(emptiedQueue) {
                // Check if we have emptied all queues
                bool allEmptied = true;

                for(const auto& queue : waypoints) {
                    emptiedQueue &= queue.empty();
                }

                // If we emptied all queues, then emit an event that we are all finished
                if(allEmptied) {
                    allQueueEnd();
                }
            }

            auto commands = std::make_unique<std::vector<messages::platform::darwin::DarwinServoCommand>>();

            // Check if there are any unexecuted motions that are in our current window and execute them if they are in range
            for(size_t i = 0; i < 20; ++i) {
                if(!waypoints[i].empty()
                   && !waypoints[i].front().executed
                   && waypoints[i].front().start < time) {

                    // We are about to execute this command
                    waypoints[i].front().executed = true;

                    // Get all of our relevant data
                    float gain = waypoints[i].front().gain;
                    float presentPosition = sensors.servo[i].presentPosition;
                    float targetPosition = waypoints[i].front().position;
                    auto end = waypoints[i].front().end;
                    auto time = NUClear::clock::now();
                    messages::platform::darwin::DarwinSensors::Servo::ID id = static_cast<messages::platform::darwin::DarwinSensors::Servo::ID>(i);

                    // If the distance we would travel is greater then 75% of pi, we have to split this waypoint.
                    // Otherwise the robot will take the "shortest" path to the goal. This will result in it potentially
                    // damaging itself as the motor moves around the wrong way.
                    float distance = utility::math::angle::difference(presentPosition, targetPosition);
                    if(distance >= M_PI * 0.75) {

                        // Create our midpoint waypoint
                        Motion midpoint;
                        midpoint.start = time + (end - time) / 2;
                        midpoint.end = waypoints[i].front().end;
                        midpoint.position = targetPosition;
                        midpoint.gain = gain;
                        midpoint.executed = false;

                        // Insert this waypoint as the next instruction
                        waypoints[i].insert(++waypoints[i].begin(), std::move(midpoint));

                        // Adjust our existing point
                        end = waypoints[i].front().end = midpoint.start;
                        targetPosition = waypoints[i].front().position = targetPosition - (distance / 2);
                    }

                    // Work out our radians per second speed to get there
                    float movingSpeed = fabs(targetPosition - presentPosition) /
                        (float((end - time).count()) / float(NUClear::clock::period::den));

                    // If it's less then 0 make it 0 (negative movings speeds would be bad)
                    movingSpeed = movingSpeed < 0 ? 0 : movingSpeed;

                    // If this moving speed is unattainable
                    if(movingSpeed > 60) {
                        // TODO some sort of log warning could be thrown here that we can't go this fast (motors can't)
                    }

                    // Add this command to our vector of commands
                    messages::platform::darwin::DarwinServoCommand command;
                    command.id = id;
                    command.goalPosition = targetPosition;
                    command.movingSpeed = movingSpeed;

                    // TODO if someone were to want to tune the PID values, this is where it would go.
                    // multiply the values by gain to get the "softness" of the motion
                    command.pGain = gain;
                    command.iGain = 0;
                    command.dGain = 0;

                    // Push this onto our list of commands
                    commands->push_back(std::move(command));
                }
            }

            // If we have commands to execute then emit them
            if(!commands->empty()) {
                emit(std::move(commands));
            }
        });

        // For single waypoints
        on<Trigger<messages::motion::ServoWaypoint>>([this](const messages::motion::ServoWaypoint& point) {

            // Make a vector of the command
            auto points = std::make_unique<std::vector<messages::motion::ServoWaypoint>>();
            points->push_back(point);
            emit<Scope::DIRECT>(std::move(points));
        });

        on<Trigger<std::vector<messages::motion::ServoWaypoint>>>([this](const std::vector<messages::motion::ServoWaypoint>& points) {
            std::lock_guard<std::mutex> waypointLock(waypointMutex);

            for(const auto& point : points) {

                // Get an easy reference to our queue
                auto& queue = waypoints[static_cast<int>(point.id)];

                // Remove all events that start after this event ends:
                // We want to remove all the events that start after our new
                // event in order to avoid running multiple scripts over the
                // top of each other.
                while(!queue.empty() && queue.back().start >= point.time) {
                    queue.pop_back();
                }

                Motion m;

                // If we have an event in the queue, then we start when this event starts otherwise we start now
                m.start = queue.empty() ? NUClear::clock::now() : queue.back().end;
                m.end = point.time;
                m.position = utility::math::angle::normalizeAngle(point.position);;
                m.gain = point.gain;
                m.executed = false;

                queue.push_back(std::move(m));
            }
        });
    }

    void MotionManager::queueEnd(size_t queue) {
        switch(queue) {
            case 0: emit(std::make_unique<messages::motion::ServoWaypointsComplete<messages::platform::darwin::DarwinSensors::Servo::ID::R_SHOULDER_PITCH>>()); break;
            case 1: emit(std::make_unique<messages::motion::ServoWaypointsComplete<messages::platform::darwin::DarwinSensors::Servo::ID::L_SHOULDER_PITCH>>()); break;
            case 2: emit(std::make_unique<messages::motion::ServoWaypointsComplete<messages::platform::darwin::DarwinSensors::Servo::ID::R_SHOULDER_ROLL>>()); break;
            case 3: emit(std::make_unique<messages::motion::ServoWaypointsComplete<messages::platform::darwin::DarwinSensors::Servo::ID::L_SHOULDER_ROLL>>()); break;
            case 4: emit(std::make_unique<messages::motion::ServoWaypointsComplete<messages::platform::darwin::DarwinSensors::Servo::ID::R_ELBOW>>()); break;
            case 5: emit(std::make_unique<messages::motion::ServoWaypointsComplete<messages::platform::darwin::DarwinSensors::Servo::ID::L_ELBOW>>()); break;
            case 6: emit(std::make_unique<messages::motion::ServoWaypointsComplete<messages::platform::darwin::DarwinSensors::Servo::ID::R_HIP_YAW>>()); break;
            case 7: emit(std::make_unique<messages::motion::ServoWaypointsComplete<messages::platform::darwin::DarwinSensors::Servo::ID::L_HIP_YAW>>()); break;
            case 8: emit(std::make_unique<messages::motion::ServoWaypointsComplete<messages::platform::darwin::DarwinSensors::Servo::ID::R_HIP_ROLL>>()); break;
            case 9: emit(std::make_unique<messages::motion::ServoWaypointsComplete<messages::platform::darwin::DarwinSensors::Servo::ID::L_HIP_ROLL>>()); break;
            case 10: emit(std::make_unique<messages::motion::ServoWaypointsComplete<messages::platform::darwin::DarwinSensors::Servo::ID::R_HIP_PITCH>>()); break;
            case 11: emit(std::make_unique<messages::motion::ServoWaypointsComplete<messages::platform::darwin::DarwinSensors::Servo::ID::L_HIP_PITCH>>()); break;
            case 12: emit(std::make_unique<messages::motion::ServoWaypointsComplete<messages::platform::darwin::DarwinSensors::Servo::ID::R_KNEE>>()); break;
            case 13: emit(std::make_unique<messages::motion::ServoWaypointsComplete<messages::platform::darwin::DarwinSensors::Servo::ID::L_KNEE>>()); break;
            case 14: emit(std::make_unique<messages::motion::ServoWaypointsComplete<messages::platform::darwin::DarwinSensors::Servo::ID::R_ANKLE_PITCH>>()); break;
            case 15: emit(std::make_unique<messages::motion::ServoWaypointsComplete<messages::platform::darwin::DarwinSensors::Servo::ID::L_ANKLE_PITCH>>()); break;
            case 16: emit(std::make_unique<messages::motion::ServoWaypointsComplete<messages::platform::darwin::DarwinSensors::Servo::ID::R_ANKLE_ROLL>>()); break;
            case 17: emit(std::make_unique<messages::motion::ServoWaypointsComplete<messages::platform::darwin::DarwinSensors::Servo::ID::L_ANKLE_ROLL>>()); break;
            case 18: emit(std::make_unique<messages::motion::ServoWaypointsComplete<messages::platform::darwin::DarwinSensors::Servo::ID::HEAD_PAN>>()); break;
            case 19: emit(std::make_unique<messages::motion::ServoWaypointsComplete<messages::platform::darwin::DarwinSensors::Servo::ID::HEAD_TILT>>()); break;
        }
    }

    void MotionManager::allQueueEnd() {
        emit(std::make_unique<messages::motion::AllServoWaypointsComplete>());
    }
}
}
}
