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

#ifndef MESSAGE_MOTION_WALKCOMMAND_H
#define MESSAGE_MOTION_WALKCOMMAND_H

#include <armadillo>
#include "utility/math/matrix/Transform2D.h"

namespace message 
{
namespace motion 
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

    using utility::math::matrix::Transform2D;

    struct WalkStarted {};
    struct WalkStopped {};

    struct WalkCommand 
    {
        //WalkCommand() = delete;
        size_t subsumptionId;           // reservation identifier for servo control
        Transform2D command;            // x and y are velocity in m/s and angle is in rads/s

        WalkCommand     (
                            size_t id, 
                            const Transform2D& command
                        ) 
        : subsumptionId(id)
        , command(command) {}
    };

    struct StopCommand 
    {
        //StopCommand() = delete;
        size_t subsumptionId;           // reservation identifier for servo control

        StopCommand     (
                            size_t id
                        ) 
        : subsumptionId(id) {}
    };

    struct NewWalkCommand 
    {
        //NewWalkCommand() = delete;
        Transform2D velocityTarget;     //

        NewWalkCommand(const Transform2D& velocityTarget)
        : velocityTarget(velocityTarget) {}
    };

    struct EnableWalkEngineCommand 
    {
        //EnableWalkEngineCommand() = delete;
        size_t subsumptionId;           // reservation identifier for servo control

        EnableWalkEngineCommand(size_t id) : subsumptionId(id) { }
    };

    struct DisableWalkEngineCommand 
    {
        //DisableWalkEngineCommand() = delete;
        size_t subsumptionId;           // reservation identifier for servo control

        DisableWalkEngineCommand(size_t id) : subsumptionId(id) { }
    };

} //motion
} //message

#endif  // MESSAGE_MOTION_WALKCOMMAND_H