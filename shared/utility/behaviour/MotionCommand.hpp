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
#ifndef UTILITY_BEHAVIOUR_MOTIONCOMMAND_HPP
#define UTILITY_BEHAVIOUR_MOTIONCOMMAND_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "message/behaviour/MotionCommand.hpp"

namespace utility::behaviour {

    using message::behaviour::MotionCommand;

    inline MotionCommand StandStill() {
        MotionCommand cmd;
        cmd.type = MotionCommand::Type::Value::STAND_STILL;
        return cmd;
    }

    inline MotionCommand WalkToState(const Eigen::Isometry2d& goalState_) {
        MotionCommand cmd;
        cmd.type       = MotionCommand::Type::Value::WALK_TO_STATE;
        cmd.goal_state = Eigen::Vector3d(goalState_.translation().x(),
                                         goalState_.translation().y(),
                                         Eigen::Rotation2Dd(goalState_.linear()).angle());
        return cmd;
    }

    inline MotionCommand BallApproach() {
        MotionCommand cmd;
        cmd.type = MotionCommand::Type::Value::BALL_APPROACH;
        return cmd;
    }

    inline MotionCommand DirectCommand(const Eigen::Isometry2d& walkCommand_) {
        MotionCommand cmd;
        cmd.type         = MotionCommand::Type::Value::DIRECT_COMMAND;
        cmd.walk_command = Eigen::Vector3d(walkCommand_.translation().x(),
                                           walkCommand_.translation().y(),
                                           Eigen::Rotation2Dd(walkCommand_.linear()).angle());
        return cmd;
    }

    inline MotionCommand RotateOnSpot(bool clockwise) {
        MotionCommand cmd;
        cmd.type      = MotionCommand::Type::Value::ROTATE_ON_SPOT;
        cmd.clockwise = clockwise;
        return cmd;
    }

    inline MotionCommand WalkToReady() {
        MotionCommand cmd;
        cmd.type = MotionCommand::Type::Value::WALK_TO_READY;
        return cmd;
    }

    inline MotionCommand RotateAroundBall() {
        MotionCommand cmd;
        cmd.type = MotionCommand::Type::Value::ROTATE_AROUND_BALL;
        return cmd;
    }

    // TODO(BehaviourTeam): Create accessor methods that throw errors if the data
    // accessed does not correspond to the command type?

    // Note: We used to use more generic goal and kickTarget types, but
    // these have been removed for simplicity.
    // // What to face towards upon reaching the goal.
    // // enum class TargetType {
    // //     WayPoint,
    // //     Ball
    // // };
    // // TargetType targetLookAt = WayPoint;
}  // namespace utility::behaviour
#endif
