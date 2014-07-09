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

#ifndef UTILITY_MATH_GEOMETRY_UNITQUATERNION_H
#define UTILITY_MATH_GEOMETRY_UNITQUATERNION_H

#include <armadillo>

namespace utility {
namespace math {
namespace geometry {

    class UnitQuaternion {
    private:
        /* @brief Constructor for non-unit quaternion for purpose of point representation
        */
        UnitQuaternion(const arma::vec3& v);

    public:
        //q stores the components of the quaternion, with real part first
        arma::vec4 q;

        UnitQuaternion operator * (const UnitQuaternion& p) const;

        /*! @brief Instantiates quat directly
        WARNING: FOR EFFICIENCY THIS ASSUMES q_ IS A UNIT QUATERNION
        @param q_ UNIT quaternion in real, imaginary order
        */
        UnitQuaternion(const arma::vec4& q_);
        /*! @brief Creates quaternion which rotates about 3D axis by angle radians
        */
        UnitQuaternion(const double& angle, const arma::vec3& axis);

        /*! @brief Gets the inverse of the quaternion
        */
        UnitQuaternion i();

        arma::vec3 rotateVector(const arma::vec3& v);

        arma::vec3 getAxis();

        double getAngle();

        /*! @brief Returns the matrix which performs the same rotation as the rotateVector method.
        When representing a basis, this transform maps points written in basis coords to points in world coords (i.e x,y,z) : B -> W
        */
        arma::mat33 getMatrix();

        /*! @brief Calls corresponding function on stored q vector
        Note: ACCESS ONLY (NO WRITING)
        */
        arma::vec rows(const uint& i, const uint& j) const;
        /*! @brief Calls corresponding function on stored q vector
        Note: ACCESS ONLY (NO WRITING)
        */
        double operator [] (const uint& i) const;

    };

}
}
}

#endif
