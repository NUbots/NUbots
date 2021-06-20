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

#ifndef UTILITY_MOTION_INVERSEKINEMATICS_HPP
#define UTILITY_MOTION_INVERSEKINEMATICS_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <chrono>
#include <cmath>
#include <limits>
#include <nuclear>
#include <vector>

#include "message/input/Sensors.hpp"
#include "message/motion/KinematicsModel.hpp"

#include "utility/behaviour/Action.hpp"
#include "utility/input/LimbID.hpp"
#include "utility/input/ServoID.hpp"
#include "utility/math/angle.hpp"
#include "utility/math/coordinates.hpp"
#include "utility/motion/ForwardKinematics.hpp"

namespace utility::motion::kinematics {

    using utility::input::LimbID;
    using utility::input::ServoID;

    /*! @brief Calculates the leg joints for a given input ankle position.
            The robot coordinate system has origin a distance DISTANCE_FROM_BODY_TO_HIP_JOINT above the midpoint
       of the hips. Robot coordinate system: x is out of the front of the robot y is left, from right shoulder
       to left z is upward, from feet to head Input ankle coordinate system: x is forward, from heel to toe y is
       left, z is normal to the plane of the foot
        @param target The target 4x4 basis matrix for the ankle
        @param isLeft Request for left leg motors or right leg motors?
        @param RobotKinematicsModel The class containing the leg model of the robot.
    */

    [[nodiscard]] std::vector<std::pair<ServoID, double>> calculateLegJoints(
        const message::motion::KinematicsModel& model,
        const Eigen::Affine3d& target,
        const LimbID& limb);

    [[nodiscard]] std::vector<std::pair<ServoID, float>> calculateLegJoints(
        const message::motion::KinematicsModel& model,
        const Eigen::Affine3f& target,
        const LimbID& limb);

    [[nodiscard]] std::vector<std::pair<ServoID, float>> calculateLegJoints(
        const message::motion::KinematicsModel& model,
        const Eigen::Affine3f& leftTarget,
        const Eigen::Affine3f& rightTarget);

    [[nodiscard]] std::vector<std::pair<ServoID, double>> calculateCameraLookJoints(
        const KinematicsModel& model,
        const Eigen::Vector3d& cameraUnitVector);

    [[nodiscard]] std::vector<std::pair<ServoID, float>> calculateHeadJoints(Eigen::Vector3f cameraUnitVector);

}  // namespace utility::motion::kinematics

#endif  // UTILITY_MOTION_INVERSEKINEMATICS_HPP
