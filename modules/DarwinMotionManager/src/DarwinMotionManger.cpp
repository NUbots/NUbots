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
        
        on<Trigger<Every<20, std::chrono::milliseconds>>>([this](const time_t&) {
            
            std::vector<messages::DarwinServoCommand>* commands = new std::vector<messages::DarwinServoCommand>();
            
            for(auto& list : waypoints) {
                // If it's time to execute this waypoint
                if(list.front().time < NUClear::clock::now()) {
                    
                }
            }
            
            // Check if we have a new command to run
            
            // Run our command as a syncwrite to darwin
            
            // Check our waypoints to run and queue up the next event
            
            // If we have removed the last event on any queue emit an event that says that this motor is finished
        });
        
        on<Trigger<std::vector<messages::ServoWaypoint>>>([this](const std::vector<messages::ServoWaypoint>& newPoints) {
            
            for(auto& waypoint : newPoints) {
                // If this waypoint is before the end point, strip points until this is the last point
                while(!waypoints[waypoint.id].empty() && waypoints[waypoint.id].back().time > waypoint.time) {
                    waypoints[waypoint.id].pop_back();
                }
                
                // Push this waypoint onto the end of the list of waypoints
                waypoints[waypoint.id].push_back(waypoint);
            }
        });
    }
}
