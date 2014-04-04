/*
 * This file is part of NUBots Utility.
 *
 * NUBots Utility is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * NUBots Utility is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with NUBots Utility.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#ifndef UTILITY_LOCALISATION_TRANSFORM_H
#define UTILITY_LOCALISATION_TRANSFORM_H

#include <armadillo>

namespace utility {
namespace localisation {
namespace transform {

    inline arma::vec2 RobotBall2FieldBall(const arma::vec2& robot_pos,
                                          const arma::vec2& robot_heading,
                                          const arma::vec2& ball_pos) {
        arma::vec2 u = arma::normalise(robot_heading);
        arma::mat22 rot = {  u[0], u[1],
                            -u[1], u[0] };
        // Rotate ball_pos by -robot_heading, then add robot_pos.
        return rot * ball_pos + robot_pos;


        // double t = -std::atan2(robot_heading[1], robot_heading[0]);
        // arma::mat22 rot = {  std::cos(t), -std::sin(t),
        //                      std::sin(t),  std::cos(t) };
        // return rot * ball_pos + robot_pos;
    }

}
}
}
#endif
