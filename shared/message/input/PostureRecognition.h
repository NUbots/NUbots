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
 * Copyright 2015 NUBots <nubots@nubots.net>
 */

#ifndef MESSAGE_INPUT_POSTURERECOGNITION_H
#define MESSAGE_INPUT_POSTURERECOGNITION_H

#include <string>

namespace message 
{
namespace input 
{
	/*
    struct [name]
    {
        [variables]
        [structure](...)
            : ...
            , ... {}
    };
    */

    struct WalkingDetected
    {
        
    };

    struct BendingDetected
    {
        
    };

    struct KickingDetected
    {
        
    };

    struct SittingDetected
    {
        
    };

    struct StandingDetected
    {
        
    };

    struct FallingDetected
    {
        // Returns a unitless severity measure, 0.0 to 1.0 [0..100%], for compensation reactions...
        double  x;      // Detects if there is some notible x  component to falling acceleration...
        double 	y;      // Detects if there is some notible y  component to falling acceleration...
        double 	z;      // Detects if there is some notible z  component to falling acceleration...
        FallingDetected ( double inX, double inY, double inZ )
            : x(inX)
            , y(inY)
            , z(inZ) {}
    };
}
}

#endif