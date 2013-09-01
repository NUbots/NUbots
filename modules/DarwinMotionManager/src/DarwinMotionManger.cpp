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

#include "DarwinMotionManager.h"

namespace modules {

    DarwinMotionManager::DarwinMotionManager(NUClear::PowerPlant* plant) : Reactor(plant) {

        on<Trigger<Every<20, std::chrono::milliseconds>>, With<messages::DarwinSensors>>([this](const time_t& time, const messages::DarwinSensors& sensors) {

            // Firstly see if there are any old motions that have expired and remove them
            for (auto& queue : waypoints) {
                while(!queue.empty() && queue.front().executed && queue.front().end < time) {
                    queue.pop_front();

                    // If we just completely  emptied this queue then emit an event to say so
                    // TODO
                }
            }

            // If we removed the final element of a queue and now all queues are empty, emit a total finish event

            auto commands = std::make_unique<std::vector<messages::DarwinServoCommand>>();

            // Check if there are any unexecuted motions that are in our current window and execute them if they are in range
            for(size_t i = 0; i < 20; ++i) {
                if(waypoints[i].empty()
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
                emit(std::move(commands));
            }
        });

        on<Trigger<messages::ServoWaypoint>>([this](const messages::ServoWaypoint& waypoint) {

            // TODO take this waypoint and append it to where it fits in the queue

            // If it is before the last event in the queue, keep removing the final element until we are back to 0

            // Then make a motion based on this element

        });
    }
}
