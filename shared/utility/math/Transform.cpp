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

#include "Transform.h"

#include <nuclear>

#include "utility/math/Rotation.h"

namespace utility {
namespace math {

    Transform::Transform() {
        eye(); // identity matrix by default
    }

    Transform::Transform(arma::vec4 q) : Transform() {
        // quaternion to rotation conversion
        // http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToMatrix/
        // http://en.wikipedia.org/wiki/Rotation_group_SO(3)#Quaternions_of_unit_norm
        submat(0,0,2,2) = Rotation(q);
    }

    Transform::Transform(arma::vec6 in) : Transform(Transform().translate(in.rows(0,2)).rotateZ(in[5]).rotateY(in[4]).rotateX(in[3])) {

    }

    Transform Transform::translate(const arma::vec3& translation) const {
        return *this * createTranslation(translation);
    }

    Transform Transform::translateX(double translation) const {
        return translate({translation, 0, 0});
    }

    Transform Transform::translateY(double translation) const {
        return translate({0, translation, 0});
    }

    Transform Transform::translateZ(double translation) const {
        return translate({0, 0, translation});
    }

    Transform Transform::rotateX(double radians) const {
        return *this * createRotationX(radians);
    }

    Transform Transform::rotateY(double radians) const {
        return *this * createRotationY(radians);
    }

    Transform Transform::rotateZ(double radians) const {
        return *this * createRotationZ(radians);
    }

    Transform Transform::worldToLocal(const Transform& reference) const {
        // http://en.wikipedia.org/wiki/Change_of_basis
        return reference.i() * (*this);
    }

    Transform Transform::localToWorld(const Transform& reference) const {
        // http://en.wikipedia.org/wiki/Change_of_basis
        return reference * (*this);
    }

    Transform Transform::i() const {
        // Create a new transform
        Transform inverseTransform;
        // Transpose the rotation submatrix (top-left 3x3), this is equivalent to taking the inverse of the rotation matrix
        inverseTransform.submat(0,0,2,2) = submat(0,0,2,2).t();
        // Multiply translation vector (top-right column vector) by the negated inverse rotation matrix
        inverseTransform.submat(0,3,2,3) = -inverseTransform.submat(0,0,2,2) * submat(0,3,2,3);
        /*if (arma::norm(inverseTransform * (*this) - arma::eye(4,4)) > 1e-10){
            NUClear::log<NUClear::WARN>("Inverse failed! Matrix is singular");
        }*/
        return inverseTransform;
    }

    Transform Transform::createTranslation(const arma::vec3& translation) {
        Transform transform;
        transform.col(3).rows(0,2) = translation;
        return transform;
    }

    Transform Transform::createRotationX(double radians) {
        Transform transform;
        transform.submat(0,0,2,2) = Rotation::createRotationX(radians);
        return transform;
    }

    Transform Transform::createRotationY(double radians) {
        Transform transform;
        transform.submat(0,0,2,2) = Rotation::createRotationY(radians);
        return transform;
    }

    Transform Transform::createRotationZ(double radians) {
        Transform transform;
        transform.submat(0,0,2,2) = Rotation::createRotationZ(radians);
        return transform;
    }

}
}
