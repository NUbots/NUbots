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

#include "Rotation3D.h"

#include <nuclear>

namespace utility {
namespace math {
namespace matrix {

    Rotation3D::Rotation() {
        eye(); // identity matrix by default
    }

    Rotation3D::Rotation(const arma::vec4& q) {
        // quaternion to rotation conversion
        // http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToMatrix/
        // http://en.wikipedia.org/wiki/Rotation_group_SO(3)#Quaternions_of_unit_norm
        *this << 1 - 2 * q[2] * q[2] - 2 * q[3] * q[3] << 2     * q[1] * q[2] - 2 * q[3] * q[0] << 2     * q[1] * q[3] + 2 * q[2] * q[0] << arma::endr
              << 2     * q[1] * q[2] + 2 * q[3] * q[0] << 1 - 2 * q[1] * q[1] - 2 * q[3] * q[3] << 2     * q[2] * q[3] - 2 * q[1] * q[0] << arma::endr
              << 2     * q[1] * q[3] - 2 * q[2] * q[0] << 2     * q[2] * q[3] + 2 * q[1] * q[0] << 1 - 2 * q[1] * q[1] - 2 * q[2] * q[2];
    }

    Rotation3D::Rotation(const arma::vec3& axis, double angle) {
        //Construct appropriate ONB:

        //Check axis not zero
        double normAxis = arma::norm(axis, 2);
        if (normAxis != 0) {
            col(0) = axis/normAxis;
        }
        else {
            NUClear::log<NUClear::WARN>("utility::math::matrix::axisAngleRotationMatrix - WARNING Zero rotation axis given");
            eye();
            return;
        }

        //Get first orthogonal vector
        col(1) = arma::vec3({0, col(0)[2], -col(0)[1]});  //orthogonal to col0, unless zero
        double col1Norm = arma::norm(col(1), 2);
        if (col1Norm == 0) {
            col(1) = arma::vec3({col(0)[1], -col(0)[0], 0});   //orthogonal to col0
            col(1) *= (1 / arma::norm(col(1), 2));
        }
        else {
            col(1) *= (1 / col1Norm);
        }

        //Get second orthogonal vector
        col(2) = arma::cross(col(0), col(1));

        *this *= Rotation3D::createRotationX(angle) * i();
    }

    Rotation3D Rotation3D::rotateX(double radians) const {
        return *this * createRotationX(radians);
    }

    Rotation3D Rotation3D::rotateY(double radians) const {
        return *this * createRotationY(radians);
    }

    Rotation3D Rotation3D::rotateZ(double radians) const {
        return *this * createRotationZ(radians);
    }

    Rotation3D Rotation3D::i() const {
        // http://en.wikipedia.org/wiki/Rotation_matrix#Multiplication
        // The inverse of a rotation matrix is its transpose, which is also a rotation matrix.
        return t();
    }


    std::pair<arma::vec3, double> Rotation3D::axisAngle() const {
        std::pair<arma::vec3, double> result;
        arma::cx_vec eigValues;
        arma::cx_mat eigVectors;
        eig_gen(eigValues, eigVectors, *this);

        for (size_t i = 0; i < eigValues.size(); i++) {
            if (std::real(eigValues[i]) == 1) {
                result.first = arma::real(eigVectors.col(i));   //Set axis of rotation for return
            }
        }
        double norm = arma::norm(result.first, 2);
        if (norm != 0) {
            result.first *= 1/norm;
        }
        else {
            NUClear::log<NUClear::ERROR>("utility::math::matrix::Rotation3D::axisAngle -  ERROR :  No rotation found");
            return result;
        }

        //Construct an ONB
        arma::vec3 s = {0, -result.first[2], result.first[1]};    //orth to result.first
        double sNorm = arma::norm(s, 2);
        if (sNorm == 0) {
            s = arma::vec({result.first[1], -result.first[0],0});
            s *= (1 / arma::norm(s, 2));
        }
        else {
            s *= (1 / sNorm);
        }
        arma::vec3 t = arma::cross(result.first, s);             //orth to both
        arma::vec3 Rs = *this * s;                               //Rotate s to calculate angle of rotation

        result.second = atan2(arma::dot(Rs, t), arma::dot(Rs, s)); //Set angle of rotation for return

        return result;  //returns axis as vec3 and angle as double
    }


    Rotation3D Rotation3D::createRotationX(double radians) {
        double c = cos(radians);
        double s = sin(radians);
        Rotation rotation;
        // http://en.wikipedia.org/wiki/Rotation_matrix#Basic_rotations
        rotation << 1 << 0 <<  0 << arma::endr
                 << 0 << c << -s << arma::endr
                 << 0 << s <<  c;
        return rotation;
    }

    Rotation3D Rotation3D::createRotationY(double radians) {
        double c = cos(radians);
        double s = sin(radians);
        Rotation rotation;
        // http://en.wikipedia.org/wiki/Rotation_matrix#Basic_rotations
        rotation <<  c << 0 << s << arma::endr
                 <<  0 << 1 << 0 << arma::endr
                 << -s << 0 << c;
        return rotation;
    }

    Rotation3D Rotation3D::createRotationZ(double radians) {
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