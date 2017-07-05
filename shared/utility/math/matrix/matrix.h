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

#ifndef UTILITY_MATH_MATRIX_H
#define UTILITY_MATH_MATRIX_H


/**
 * Matrix related helper methods
 *
 * @author Brendan Annable
 */

namespace utility {
namespace math {
    namespace matrix {


        /**
         * @brief Returns an orthogonal vec3 to a given vec3
         * See: http://a.pomf.se/egicug.pdf
         * Efficient Construction of Perpendicular Vectors without Branching
         * Michael M. Stark
         */
        Eigen::Vector3d orthogonal(const Eigen::Vector3d& v) {
            const unsigned int uyx = std::signbit(std::abs(v[0]) - std::abs(v[1]));
            const unsigned int uzx = std::signbit(std::abs(v[0]) - std::abs(v[2]));
            const unsigned int uzy = std::signbit(std::abs(v[1]) - std::abs(v[2]));
            const unsigned int xm  = uyx & uzx;
            const unsigned int ym  = (1 ^ xm) & uzy;
            const unsigned int zm  = 1 ^ (xm & ym);
            return {zm * v[1] - ym * v[2], xm * v[2] - zm * v[0], ym * v[0] - xm * v[1]};
        }

        /**
         * @brief An alternative method for returning a orthogonal vec3 to a given vec3
         * See: http://lolengine.net/blog/2013/09/21/picking-orthogonal-vector-combing-coconuts
         */
        Eigen::Vector3d orthogonal2(const Eigen::Vector3d& v) {
            return std::abs(v[0]) > std::abs(v[2]) ? Eigen::Vector3d(-v[1], v[0], 0) : Eigen::Vector3d(0, -v[2], v[1]);
        }

        /**
         * @brief Returns an arbitary orthonormal vec3 to the given vec3
         */
        Eigen::Vector3d orthonormal(const Eigen::Vector3d& v) {
            auto u = orthogonal(v);
            return u.normalized();
        }
    }
}
}
#endif  // UTILITY_MATH_MATRIX_H
