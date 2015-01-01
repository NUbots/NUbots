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

#ifndef UTILITY_MATH_TRANSFORM_H
#define UTILITY_MATH_TRANSFORM_H

#include <armadillo>

namespace utility {
namespace math {

    /**
     * TODO
     *
     * @author Brendan Annable
     */
    class Transform : public arma::mat44 {
        using arma::mat44::mat44; // inherit constructors

        public:
            Transform();
            Transform& translate(const arma::vec3& translation);
            Transform& translateX(double translation);
            Transform& translateY(double translation);
            Transform& translateZ(double translation);
            Transform& rotateX(double radians);
            Transform& rotateY(double radians);
            Transform& rotateZ(double radians);
            Transform& worldToLocal(const Transform& local);
            Transform& localToWorld(const Transform& local);
    };

}  // math
}  // utility

#endif  // UTILITY_MATH_TRANSFORM_H

