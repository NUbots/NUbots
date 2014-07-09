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

#include <armadillo>
#include "utility/math/matrix.h"

namespace utility {
namespace localisation {
namespace transform {

    inline arma::vec RobotToWorldTransform(const arma::vec& robot_pos,
                                           const arma::vec& robot_heading,
                                           const arma::vec& relative_ball_pos) {
        arma::vec u = arma::normalise(robot_heading);
        arma::mat rot;
        rot <<  u[0] << -u[1] << arma::endr
            <<  u[1] <<  u[0];
        // Rotate relative_ball_pos by robot_heading, then add robot_pos.
        return rot * relative_ball_pos + robot_pos;
    }

    inline arma::vec WorldToRobotTransform(const arma::vec& robot_pos,
                                           const arma::vec& robot_heading,
                                           const arma::vec& field_ball_pos) {
        arma::vec u = arma::normalise(robot_heading);
        arma::mat rot;
        rot <<  u[0] <<  u[1] << arma::endr
            << -u[1] <<  u[0];
        // Subtract robot_pos, then rotate relative_ball_pos by -robot_heading.
        return rot * (field_ball_pos - robot_pos);
    }

    inline arma::vec RobotToWorldTransform(const arma::vec& robot_pos,
                                           const double& robot_heading,
                                           const arma::vec& relative_ball_pos) {
        arma::mat rot = utility::math::matrix::rotationMatrix(robot_heading);
        // Rotate relative_ball_pos by robot_heading, then add robot_pos.
        return rot * relative_ball_pos + robot_pos;
    }

    inline arma::vec WorldToRobotTransform(const arma::vec& robot_pos,
                                           const double& robot_heading,
                                           const arma::vec& field_ball_pos) {
        arma::mat rot = utility::math::matrix::rotationMatrix(-robot_heading);
        // Subtract robot_pos, then rotate relative_ball_pos by -robot_heading.
        return rot * (field_ball_pos - robot_pos);
    }

}
}
}
#endif
