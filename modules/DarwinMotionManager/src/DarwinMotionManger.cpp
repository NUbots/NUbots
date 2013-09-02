/*
 * This file is part of DarwinMotionManager.
 *
 * DarwinMotionManager is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * DarwinMotionManager is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with DarwinMotionManager.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 Trent Houliston <trent@houliston.me>
 */

#include "messages/ServoWaypoint.h"

#include "DarwinMotionManager.h"

namespace modules {

    DarwinMotionManager::DarwinMotionManager(NUClear::PowerPlant* plant) : Reactor(plant) {

        on<Trigger<Every<20, std::chrono::milliseconds>>, With<messages::DarwinSensors>>([this](const time_t& time, const messages::DarwinSensors& sensors) {

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

                if(allEmptied) {
                    allQueueEnd();
                }
            }

            auto commands = std::make_unique<std::vector<messages::DarwinServoCommand>>();

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
                    messages::DarwinSensors::Servo::ID id = static_cast<messages::DarwinSensors::Servo::ID>(i);

                    // Work out our radians per second speed to get there
                    float movingSpeed = abs(targetPosition - presentPosition) /
                    (float((waypoints[i].front().end - time).count()) / float(NUClear::clock::period::den));

                    // If it's less then 0 make it 0 (negative movings speeds would be bad)
                    movingSpeed = movingSpeed < 0 ? 0 : movingSpeed;

                    // If this moving speed is unattainable
                    if(movingSpeed > 60) {
                        // TODO some sort of log warning could be thrown here that we can't go this fast (motors can't)
                    }

                    // Add this command to our vector of commands
                    messages::DarwinServoCommand command;
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
                std::cout << "Sending commands to servos" << std::endl;
                //emit(std::move(commands));

                std::cout << "Sent commands to servos" << std::endl;
            }
        });

        // For single waypoints
        on<Trigger<messages::ServoWaypoint>>([this](const messages::ServoWaypoint& waypoint) {

            // Make a vector of the command
            auto waypoints = std::make_unique<std::vector<messages::ServoWaypoint>>();
            waypoints->push_back(waypoint);
            emit(std::move(waypoints));
        });

        on<Trigger<std::vector<messages::ServoWaypoint>>>([this](const std::vector<messages::ServoWaypoint>& points) {
            for(const auto& point : points) {

                // Get an easy reference to our queue
                auto& queue = waypoints[static_cast<int>(point.id)];

                // Remove all events that start after this event ends
                while(!queue.empty() && queue.back().start <= point.time) {
                    queue.pop_back();
                }

                // Add this point to our queue
                Motion m;
                m.end = point.time;
                m.gain = point.gain;
                m.position = point.position;

                // If we have an event in the queue, then we start when this event starts otherwise we start now
                m.start = queue.empty() ? NUClear::clock::now() : queue.back().end;
                m.executed = false;

                waypoints[static_cast<int>(point.id)].push_back(std::move(m));
            }
        });
    }

    void DarwinMotionManager::queueEnd(size_t queue) {
        switch(queue) {
            case 0: emit(std::make_unique<messages::ServoWaypointsComplete<messages::DarwinSensors::Servo::ID::R_SHOULDER_PITCH>>()); break;
            case 1: emit(std::make_unique<messages::ServoWaypointsComplete<messages::DarwinSensors::Servo::ID::L_SHOULDER_PITCH>>()); break;
            case 2: emit(std::make_unique<messages::ServoWaypointsComplete<messages::DarwinSensors::Servo::ID::R_SHOULDER_ROLL>>()); break;
            case 3: emit(std::make_unique<messages::ServoWaypointsComplete<messages::DarwinSensors::Servo::ID::L_SHOULDER_ROLL>>()); break;
            case 4: emit(std::make_unique<messages::ServoWaypointsComplete<messages::DarwinSensors::Servo::ID::R_ELBOW>>()); break;
            case 5: emit(std::make_unique<messages::ServoWaypointsComplete<messages::DarwinSensors::Servo::ID::L_ELBOW>>()); break;
            case 6: emit(std::make_unique<messages::ServoWaypointsComplete<messages::DarwinSensors::Servo::ID::R_HIP_YAW>>()); break;
            case 7: emit(std::make_unique<messages::ServoWaypointsComplete<messages::DarwinSensors::Servo::ID::L_HIP_YAW>>()); break;
            case 8: emit(std::make_unique<messages::ServoWaypointsComplete<messages::DarwinSensors::Servo::ID::R_HIP_ROLL>>()); break;
            case 9: emit(std::make_unique<messages::ServoWaypointsComplete<messages::DarwinSensors::Servo::ID::L_HIP_ROLL>>()); break;
            case 10: emit(std::make_unique<messages::ServoWaypointsComplete<messages::DarwinSensors::Servo::ID::R_HIP_PITCH>>()); break;
            case 11: emit(std::make_unique<messages::ServoWaypointsComplete<messages::DarwinSensors::Servo::ID::L_HIP_PITCH>>()); break;
            case 12: emit(std::make_unique<messages::ServoWaypointsComplete<messages::DarwinSensors::Servo::ID::R_KNEE>>()); break;
            case 13: emit(std::make_unique<messages::ServoWaypointsComplete<messages::DarwinSensors::Servo::ID::L_KNEE>>()); break;
            case 14: emit(std::make_unique<messages::ServoWaypointsComplete<messages::DarwinSensors::Servo::ID::R_ANKLE_PITCH>>()); break;
            case 15: emit(std::make_unique<messages::ServoWaypointsComplete<messages::DarwinSensors::Servo::ID::L_ANKLE_PITCH>>()); break;
            case 16: emit(std::make_unique<messages::ServoWaypointsComplete<messages::DarwinSensors::Servo::ID::R_ANKLE_ROLL>>()); break;
            case 17: emit(std::make_unique<messages::ServoWaypointsComplete<messages::DarwinSensors::Servo::ID::L_ANKLE_ROLL>>()); break;
            case 18: emit(std::make_unique<messages::ServoWaypointsComplete<messages::DarwinSensors::Servo::ID::HEAD_PAN>>()); break;
            case 19: emit(std::make_unique<messages::ServoWaypointsComplete<messages::DarwinSensors::Servo::ID::HEAD_TILT>>()); break;
        }
    }

    void DarwinMotionManager::allQueueEnd() {
        emit(std::make_unique<messages::AllServoWaypointsComplete>());
    }
}
