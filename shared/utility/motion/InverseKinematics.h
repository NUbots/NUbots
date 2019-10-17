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

#ifndef UTILITY_MOTION_INVERSEKINEMATICS_H
#define UTILITY_MOTION_INVERSEKINEMATICS_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <armadillo>
#include <chrono>
#include <cmath>
#include <limits>
#include <nuclear>
#include <vector>

#include "message/input/Sensors.h"
#include "message/motion/KinematicsModel.h"
#include "utility/behaviour/Action.h"
#include "utility/input/LimbID.h"
#include "utility/input/ServoID.h"
#include "utility/math/angle.h"
#include "utility/math/coordinates.h"
#include "utility/math/matrix/Transform3D.h"
#include "utility/motion/ForwardKinematics.h"

namespace utility {
namespace motion {
    namespace kinematics {

        using LimbID  = utility::input::LimbID;
        using ServoID = utility::input::ServoID;

        /*! @brief Calculates the leg joints for a given input ankle position.
                The robot coordinate system has origin a distance DISTANCE_FROM_BODY_TO_HIP_JOINT above the midpoint of
           the hips.
                Robot coordinate system:
                            x is out of the front of the robot
                            y is left, from right shoulder to left
                            z is upward, from feet to head
                Input ankle coordinate system:
                            x is forward, from heel to toe
                            y is left,
                            z is normal to the plane of the foot
            @param target The target 4x4 basis matrix for the ankle
            @param isLeft Request for left leg motors or right leg motors?
            @param RobotKinematicsModel The class containing the leg model of the robot.
        */

        bool legPoseValid(const message::motion::KinematicsModel& model,
                          utility::math::matrix::Transform3D target,
                          LimbID limb);

        std::vector<std::pair<ServoID, float>> calculateLegJoints(const message::motion::KinematicsModel& model,
                                                                  utility::math::matrix::Transform3D target,
                                                                  LimbID limb);
        std::vector<std::pair<ServoID, double>> calculateLegJoints(const message::motion::KinematicsModel& model,
                                                                   const Eigen::Affine3d& target,
                                                                   const LimbID& limb);

        std::vector<std::pair<ServoID, float>> calculateLegJoints(const message::motion::KinematicsModel& model,
                                                                  utility::math::matrix::Transform3D leftTarget,
                                                                  utility::math::matrix::Transform3D rightTarget);

        std::vector<std::pair<ServoID, float>> calculateLegJointsTeamDarwin(
            const message::motion::KinematicsModel& model,
            utility::math::matrix::Transform3D target,
            LimbID limb);

        std::vector<std::pair<ServoID, float>> calculateLegJointsTeamDarwin(
            const message::motion::KinematicsModel& model,
            utility::math::matrix::Transform3D leftTarget,
            utility::math::matrix::Transform3D rightTarget);

        std::vector<std::pair<ServoID, float>> calculateCameraLookJoints(const message::motion::KinematicsModel& model,
                                                                         arma::vec3 cameraUnitVector);
        std::vector<std::pair<ServoID, double>> calculateCameraLookJoints(const message::motion::KinematicsModel& model,
                                                                          const Eigen::Vector3d& cameraUnitVector);

        std::vector<std::pair<ServoID, float>> calculateHeadJoints(arma::vec3 cameraUnitVector);

        arma::vec2 calculateHeadJointsToLookAt(arma::vec3 groundPoint,
                                               const utility::math::matrix::Transform3D& camToGround,
                                               const utility::math::matrix::Transform3D& bodyToGround);

        arma::vec2 headAnglesToSeeGroundPoint(const arma::vec2& gpos, const message::input::Sensors& sensors);

        std::vector<std::pair<ServoID, float>> setHeadPoseFromFeet(
            const message::motion::KinematicsModel& model,
            const utility::math::matrix::Transform3D& cameraToFeet,
            const float& footSeparation);

        std::vector<std::pair<ServoID, float>> setArm(const message::motion::KinematicsModel& model,
                                                      const arma::vec3& pos,
                                                      bool left,
                                                      int number_of_iterations = 300,
                                                      arma::vec3 angleHint     = arma::zeros(3));

        std::vector<std::pair<ServoID, float>> setArmApprox(const message::motion::KinematicsModel& model,
                                                            const arma::vec3& pos,
                                                            bool left);


    }  // namespace kinematics
}  // namespace motion
}  // namespace utility

#endif  // UTILITY_MOTION_INVERSEKINEMATICS_H
