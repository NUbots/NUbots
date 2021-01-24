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

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "message/behaviour/MotionCommand.hpp"

#include "utility/math/matrix/Transform2D.hpp"
#include "utility/support/eigen_armadillo.hpp"

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
        cmd.goalState = convert(goalState_);
        return cmd;
    }

    inline MotionCommand WalkToState(const Eigen::Affine2d& goalState_) {
        MotionCommand cmd;
        cmd.type      = MotionCommand::Type::Value::WalkToState;
        cmd.goalState = Eigen::Vector3d(goalState_.translation().x(),
                                        goalState_.translation().y(),
                                        Eigen::Rotation2Dd(goalState_.linear()).angle());
        return cmd;
    }

    inline MotionCommand BallApproach(const Eigen::Vector2d kickTarget_) {
        MotionCommand cmd;
        cmd.type       = MotionCommand::Type::Value::BallApproach;
        cmd.kickTarget = kickTarget_;
        return cmd;
    }

    inline MotionCommand DirectCommand(Transform2D walkCommand_) {
        MotionCommand cmd;
        cmd.type        = MotionCommand::Type::Value::DirectCommand;
        cmd.walkCommand = convert(walkCommand_);
        return cmd;
    }

    inline MotionCommand DirectCommand(const Eigen::Affine2d& walkCommand_) {
        MotionCommand cmd;
        cmd.type        = MotionCommand::Type::Value::DirectCommand;
        cmd.walkCommand = Eigen::Vector3d(walkCommand_.translation().x(),
                                          walkCommand_.translation().y(),
                                          Eigen::Rotation2Dd(walkCommand_.linear()).angle());
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
