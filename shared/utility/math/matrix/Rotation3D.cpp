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

#include "Rotation.h"

namespace utility {
namespace math {
namespace matrix {

    Rotation<3>::Rotation() {
        eye(); // identity matrix by default
    }

    Rotation<3>::Rotation(arma::vec4 q) {
        // quaternion to rotation conversion
        // http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToMatrix/
        // http://en.wikipedia.org/wiki/Rotation_group_SO(3)#Quaternions_of_unit_norm
        *this << 1 - 2 * q[2] * q[2] - 2 * q[3] * q[3] << 2     * q[1] * q[2] - 2 * q[3] * q[0] << 2     * q[1] * q[3] + 2 * q[2] * q[0] << arma::endr
              << 2     * q[1] * q[2] + 2 * q[3] * q[0] << 1 - 2 * q[1] * q[1] - 2 * q[3] * q[3] << 2     * q[2] * q[3] - 2 * q[1] * q[0] << arma::endr
              << 2     * q[1] * q[3] - 2 * q[2] * q[0] << 2     * q[2] * q[3] + 2 * q[1] * q[0] << 1 - 2 * q[1] * q[1] - 2 * q[2] * q[2];
    }

    Rotation<3> Rotation<3>::rotateX(double radians) const {
        return *this * createRotationX(radians);
    }

    Rotation<3> Rotation<3>::rotateY(double radians) const {
        return *this * createRotationY(radians);
    }

    Rotation<3> Rotation<3>::rotateZ(double radians) const {
        return *this * createRotationZ(radians);
    }

    Rotation<3> Rotation<3>::i() const {
        // http://en.wikipedia.org/wiki/Rotation_matrix#Multiplication
        // The inverse of a rotation matrix is its transpose, which is also a rotation matrix.
        return t();
    }

    Rotation<3> Rotation<3>::createRotationX(double radians) {
        double c = cos(radians);
        double s = sin(radians);
        Rotation rotation;
        // http://en.wikipedia.org/wiki/Rotation_matrix#Basic_rotations
        rotation << 1 << 0 <<  0 << arma::endr
                 << 0 << c << -s << arma::endr
                 << 0 << s <<  c;
        return rotation;
    }

    Rotation<3> Rotation<3>::createRotationY(double radians) {
        double c = cos(radians);
        double s = sin(radians);
        Rotation rotation;
        // http://en.wikipedia.org/wiki/Rotation_matrix#Basic_rotations
        rotation <<  c << 0 << s << arma::endr
                 <<  0 << 1 << 0 << arma::endr
                 << -s << 0 << c;
        return rotation;
    }

    Rotation<3> Rotation<3>::createRotationZ(double radians) {
        double c = cos(radians);
        double s = sin(radians);
        Rotation rotation;
        // http://en.wikipedia.org/wiki/Rotation_matrix#Basic_rotations
        rotation << c << -s << 0 << arma::endr
                 << s <<  c << 0 << arma::endr
                 << 0 <<  0 << 1;
        return rotation;
    }

}
}
}