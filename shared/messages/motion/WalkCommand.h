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

#ifndef MESSAGES_MOTION_WALKCOMMAND_H
#define MESSAGES_MOTION_WALKCOMMAND_H

#include <armadillo>

#include "utility/math/matrix/Transform2D.h"

namespace messages {
namespace motion {

    using utility::math::matrix::Transform2D;

    struct WalkCommand {
        WalkCommand() { }
        WalkCommand(size_t id, Transform2D command_) : subsumptionId(id), command(command_) { }
        size_t subsumptionId = 1;

        // x and y are velocity in m/s and angle is in rads/s
        utility::math::matrix::Transform2D command;
    };

    struct WalkStartCommand {
        WalkStartCommand() { }
        WalkStartCommand(size_t id) : subsumptionId(id) { }
        size_t subsumptionId = 1;
    };

    struct WalkStopCommand {
        WalkStopCommand() { }
        WalkStopCommand(size_t id) : subsumptionId(id) { }
        size_t subsumptionId = 1;
    };
    struct WalkStopped {
    };
    
    struct EnableWalkEngineCommand {
        EnableWalkEngineCommand() { }
        EnableWalkEngineCommand(size_t id) : subsumptionId(id) { }
        size_t subsumptionId = 1;
    };
    struct DisableWalkEngineCommand {
        DisableWalkEngineCommand() { }
        DisableWalkEngineCommand(size_t id) : subsumptionId(id) { }
        size_t subsumptionId = 1;
    };
}
}

#endif  // MESSAGES_MOTION_WALKCOMMAND_H