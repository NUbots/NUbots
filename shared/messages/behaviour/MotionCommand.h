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

#ifndef MESSAGES_BEHAVIOUR_MOTIONCOMMAND_H
#define MESSAGES_BEHAVIOUR_MOTIONCOMMAND_H

#include <armadillo>
#include "utility/math/matrix/Transform2D.h"

namespace messages {
namespace behaviour {

    using utility::math::matrix::Transform2D;

    struct MotionCommand {

        // Defines the possible types of motion command:
        enum class Type {
            StandStill,   // Stop moving and just stand still.
            WalkToState,  // Walk to a given position and heading on the field, avoiding obstacles.
            BallApproach, // Approach the ball, ready to perform a forward kick toward the given kickTarget. Avoids obstacles.
            DirectCommand // Stop all current motion and directly send the given WalkCommand to the WalkEngine.
        };

        // The type of this motion command:
        Type command = StandStill;

        // Required data for WalkToState command:
        Transform2D goalState = {0, 0, 0};

        // Required data for WalkToBall command:
        arma::vec2 kickTarget = {0, 0};

        // Required data for DirectCommand command:
        Transform2D walkCommand = {0, 0, 0};

        // Note: We used to use more generic goal and kickTarget types, but
        // these have been removed for simplicity.
        // // What to face towards upon reaching the goal.
        // // enum class TargetType {
        // //     WayPoint,
        // //     Ball
        // // };
        // // TargetType targetLookAt = WayPoint;
    };
}
}

#endif
