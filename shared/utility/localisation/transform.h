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

#ifndef UTILITY_LOCALISATION_TRANSFORM_H
#define UTILITY_LOCALISATION_TRANSFORM_H

#include "utility/math/matrix/Rotation2D.h"
#include "utility/math/matrix/Rotation3D.h"
#include "utility/math/coordinates.h"

namespace utility {
namespace localisation {
namespace transform {

    inline Eigen::VectorXd RobotToWorldTransform(const arma::vec& robot_pos,
                                           const arma::vec& robot_heading,
                                           const arma::vec& relative_ball_pos) {
        Eigen::VectorXd u = robot_heading.normalize();
        arma::mat rot;
        rot <<  u[0] << -u[1] << arma::endr
            <<  u[1] <<  u[0];
        // Rotate relative_ball_pos by robot_heading, then add robot_pos.
        return rot * relative_ball_pos + robot_pos;
    }

    inline Eigen::VectorXd WorldToRobotTransform(const arma::vec& robot_pos,
                                           const arma::vec& robot_heading,
                                           const arma::vec& field_ball_pos) {
        Eigen::VectorXd u = robot_heading.normalize();
        arma::mat rot;
        rot <<  u[0] <<  u[1] << arma::endr
            << -u[1] <<  u[0];
        // Subtract robot_pos, then rotate relative_ball_pos by -robot_heading.
        return rot * (field_ball_pos - robot_pos);
    }

    inline Eigen::VectorXd RobotToWorldTransform(const arma::vec& robot_pos,
                                           const double& robot_heading,
                                           const arma::vec& relative_ball_pos) {
        arma::mat rot = math::matrix::Rotation2D::createRotation(robot_heading);
        // Rotate relative_ball_pos by robot_heading, then add robot_pos.
        return rot * relative_ball_pos + robot_pos;
    }

    inline Eigen::VectorXd WorldToRobotTransform(const arma::vec& robot_pos,
                                           const double& robot_heading,
                                           const arma::vec& field_ball_pos) {
        arma::mat rot = math::matrix::Rotation2D::createRotation(-robot_heading);
        // Subtract robot_pos, then rotate relative_ball_pos by -robot_heading.
        return rot * (field_ball_pos - robot_pos);
    }

    inline Eigen::VectorXd SphericalRobotObservation(
            const arma::vec& robot_pos,
            const double& robot_heading,
            const Eigen::Vector3d& actual_position) {
        auto actual_pos_robot_2d = WorldToRobotTransform(robot_pos,
                                                     robot_heading,
                                                     actual_position.rows(0, 1));
        auto actual_pos_robot_3d = Eigen::Vector3d(actual_pos_robot_2d(0),
                                           actual_pos_robot_2d(1),
                                           actual_position(2));

        auto obs = utility::math::coordinates::cartesianToSpherical(actual_pos_robot_3d);

        return obs;
    }

    inline Eigen::Vector2d ImuToWorldHeadingTransform(double imuOffset, math::matrix::Rotation3D orientation) {
        math::matrix::Rotation3D imuRotation = math::matrix::Rotation3D::createRotationZ(imuOffset);
        Eigen::Vector3d worldRobotHeading = imuRotation * arma::mat(orientation.i()).col(0);
        return worldRobotHeading.rows(0, 1).normalize();
    }

    inline Eigen::VectorXd SphericalRobotObservation(
            const arma::vec& robot_pos,
            const Eigen::Vector2d& robot_heading,
            const Eigen::Vector3d& actual_position) {
        auto actual_pos_robot_2d = WorldToRobotTransform(robot_pos,
                                                     robot_heading,
                                                     actual_position.rows(0, 1));
        auto actual_pos_robot_3d = Eigen::Vector3d(actual_pos_robot_2d(0),
                                           actual_pos_robot_2d(1),
                                           actual_position(2));

        auto obs = utility::math::coordinates::cartesianToSpherical(actual_pos_robot_3d);

        return obs;
    }

}
}
}
#endif
