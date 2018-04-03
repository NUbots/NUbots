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
 * Copyright 2013 NUbots <nubots@nubots.net>
 */
#ifndef UTILITY_BEHAVIOUR_MOTIONCOMMAND_H
#define UTILITY_BEHAVIOUR_MOTIONCOMMAND_H

#include "message/behaviour/MotionCommand.h"

#include "utility/math/matrix/Transform2D.h"
#include "utility/support/eigen_armadillo.h"

namespace utility {
namespace behaviour {

    using message::behaviour::MotionCommand;
    using utility::math::matrix::Transform2D;

    inline MotionCommand StandStill() {
        MotionCommand cmd;
        cmd.type = MotionCommand::Type::Value::StandStill;
        return cmd;
    }

    inline MotionCommand WalkToState(Transform2D goalState_) {
        MotionCommand cmd;
        cmd.type      = MotionCommand::Type::Value::WalkToState;
        cmd.goalState = convert<double, 3>(goalState_);
        return cmd;
    }

    inline MotionCommand BallApproach(arma::vec2 kickTarget_) {
        MotionCommand cmd;
        cmd.type       = MotionCommand::Type::Value::BallApproach;
        cmd.kickTarget = convert<double, 2>(kickTarget_);
        return cmd;
    }

    inline MotionCommand DirectCommand(Transform2D walkCommand_) {
        MotionCommand cmd;
        cmd.type        = MotionCommand::Type::Value::DirectCommand;
        cmd.walkCommand = convert<double, 3>(walkCommand_);
        return cmd;
    }

    // TODO: Create accessor methods that throw errors if the data
    // accessed does not correspond to the command type?

    // Note: We used to use more generic goal and kickTarget types, but
    // these have been removed for simplicity.
    // // What to face towards upon reaching the goal.
    // // enum class TargetType {
    // //     WayPoint,
    // //     Ball
    // // };
    // // TargetType targetLookAt = WayPoint;
}  // namespace behaviour
}  // namespace utility

#endif
