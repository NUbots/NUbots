/*
 * This file is part of the NUbots Codebase.
 *
 * The NUbots Codebase is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The NUbots Codebase is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the NUbots Codebase.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#ifndef MESSAGES_BEHAVIOUR_WALKSTRATEGY_H
#define MESSAGES_BEHAVIOUR_WALKSTRATEGY_H


namespace messages {
    namespace behaviour {
        
        //define a target: x/y waypoint in field coords; robot we are tracking (for future use); ball position (updated by localisation).
        enum class WalkTarget {
            WayPoint = 0,
            Robot = 1,
            Ball = 2
        };
        
        //this is the type of walk command given: stand still; approach from a direction (for kicking);
        //walk to a point (ie initial positioning); omni-directional positioning (face heading direction while moving to position ie goalkeeper).
        enum class WalkApproach {
            StandStill = 0,
            ApproachFromDirection = 1,
            WalkToPoint = 2,
            OmnidirectionalReposition = 3,
        };

        struct WalkStrategy {
            
            //the target waypoint in x/y field coordinates if applicable
            arma::vec2 target;
            
            //the point to face in x/y field coordinates at the destination if applicable
            arma::vec2 heading;
            
            //define where to walk to (point, robot or ball)
            WalkTarget targetPositionType;
            
            //define where to face (point, robot or ball)
            WalkTarget targetHeadingType;
            
            //define the type of walk to use
            WalkApproach walkMovementType;
        };
    }
}

#endif
