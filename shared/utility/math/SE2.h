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

#ifndef UTILITY_MATH_SE2_H
#define UTILITY_MATH_SE2_H

#include <armadillo>

namespace utility {
namespace math {

    /**
     * Holds a vec3 assuming it is on the form {x, y, angle}
     *
     * See: Special Euclidean group SE(2).
     * http://en.wikipedia.org/wiki/Euclidean_group
     *
     * @author Brendan Annable
     */
    class SE2 : public arma::vec3 {
        using arma::vec3::vec3; // inherit constructors

        public:
            /**
             * Local to world transform
             *
             * Transforms the current vector from local space relative to poseRelative, to world/global space
             */
            SE2 localToWorld(const SE2& poseRelative) const;

            /**
             * World to local transform
             *
             * Transforms the current vector from world/global space to be relative to poseGlobal
             */
            SE2 worldToLocal(const SE2& poseGlobal) const;

            /**
             * Interpolate between itself and given target vector
             *
             * @param t A value between 0-1 to interpolate between the two,
             * outside these bounds will extrapolate
             * @param target The target vector
             * @return The interpolated vector
             */
            SE2 se2Interpolate(double t, const SE2& target) const;

            /**
             * Convert the vector into a 4x4 basis matrix
             * @return A 4v4 basis matrix
             */
            arma::mat44 toMatrix() const;

            inline double x()     const { return at(0); }
            inline double y()     const { return at(1); }
            inline double angle() const { return at(2); }
    };

}  // math
}  // utility

#endif  // UTILITY_MATH_SE2_H

