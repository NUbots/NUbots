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

#ifndef UTILITY_LOCALISATION_TRANSFORM_H
#define UTILITY_LOCALISATION_TRANSFORM_H

#include <armadillo>
#include "utility/math/coordinates.h"
#include "utility/math/matrix/Rotation2D.h"
#include "utility/math/matrix/Rotation3D.h"
#include "utility/math/matrix/Transform3D.h"

namespace utility {
namespace localisation {
    namespace transform {

        inline utility::math::matrix::Transform3D LocalisationStateToMatrix(const arma::vec3& state) {
            utility::math::matrix::Transform3D Hfw;
            Hfw.translation() = arma::vec3{state[0], state[1], 0};
            Hfw               = Hfw.rotateZ(state[2]);
            return Hfw;
        }

        inline arma::vec3 MatrixToLocalisationState(const utility::math::matrix::Transform3D& m) {
            utility::math::matrix::AxisAngle ax = m.rotation().axisAngle();
            if (!(arma::approx_equal(ax.first, arma::vec3({0, 0, 1}), "absdiff", 0.00001)
                  || arma::approx_equal(ax.first, arma::vec3({0, 0, -1}), "absdiff", 0.00001))) {
                throw std::runtime_error("transform.h MatrixToLocalisationState - matrix must be a z rotation");
            }
            return arma::vec3({m.translation()[0], m.translation()[1], ax.second});
        }

        inline arma::vec RobotToWorldTransform(const arma::vec& robot_pos,
                                               const arma::vec& robot_heading,
                                               const arma::vec& relative_ball_pos) {
            arma::vec u = arma::normalise(robot_heading);
            arma::mat rot;
            rot << u[0] << -u[1] << arma::endr << u[1] << u[0];
            // Rotate relative_ball_pos by robot_heading, then add robot_pos.
            return rot * relative_ball_pos + robot_pos;
        }

        inline arma::vec WorldToRobotTransform(const arma::vec& robot_pos,
                                               const arma::vec& robot_heading,
                                               const arma::vec& field_ball_pos) {
            arma::vec u = arma::normalise(robot_heading);
            arma::mat rot;
            rot << u[0] << u[1] << arma::endr << -u[1] << u[0];
            // Subtract robot_pos, then rotate relative_ball_pos by -robot_heading.
            return rot * (field_ball_pos - robot_pos);
        }

        inline arma::vec RobotToWorldTransform(const arma::vec& robot_pos,
                                               const double& robot_heading,
                                               const arma::vec& relative_ball_pos) {
            arma::mat rot = math::matrix::Rotation2D::createRotation(robot_heading);
            // Rotate relative_ball_pos by robot_heading, then add robot_pos.
            return rot * relative_ball_pos + robot_pos;
        }

        inline arma::vec WorldToRobotTransform(const arma::vec& robot_pos,
                                               const double& robot_heading,
                                               const arma::vec& field_ball_pos) {
            arma::mat rot = math::matrix::Rotation2D::createRotation(-robot_heading);
            // Subtract robot_pos, then rotate relative_ball_pos by -robot_heading.
            return rot * (field_ball_pos - robot_pos);
        }

        inline arma::vec SphericalRobotObservation(const arma::vec& robot_pos,
                                                   const double& robot_heading,
                                                   const arma::vec3& actual_position) {
            auto actual_pos_robot_2d = WorldToRobotTransform(robot_pos, robot_heading, actual_position.rows(0, 1));
            auto actual_pos_robot_3d = arma::vec3({actual_pos_robot_2d(0), actual_pos_robot_2d(1), actual_position(2)});

            auto obs = utility::math::coordinates::cartesianToSpherical(actual_pos_robot_3d);

            return obs;
        }

        inline arma::vec2 ImuToWorldHeadingTransform(double imuOffset, math::matrix::Rotation3D orientation) {
            math::matrix::Rotation3D imuRotation = math::matrix::Rotation3D::createRotationZ(imuOffset);
            arma::vec3 worldRobotHeading         = imuRotation * arma::mat(orientation.i()).col(0);
            return arma::normalise(worldRobotHeading.rows(0, 1));
        }

        inline arma::vec SphericalRobotObservation(const arma::vec& robot_pos,
                                                   const arma::vec2& robot_heading,
                                                   const arma::vec3& actual_position) {
            auto actual_pos_robot_2d = WorldToRobotTransform(robot_pos, robot_heading, actual_position.rows(0, 1));
            auto actual_pos_robot_3d = arma::vec3({actual_pos_robot_2d(0), actual_pos_robot_2d(1), actual_position(2)});

            auto obs = utility::math::coordinates::cartesianToSpherical(actual_pos_robot_3d);

            return obs;
        }
    }  // namespace transform
}  // namespace localisation
}  // namespace utility
#endif
