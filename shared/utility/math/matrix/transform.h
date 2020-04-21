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
 * Copyright 2013 NUbots <nubots@nubots.net>
 */

#ifndef UTILITY_MATH_TRANSFORM_H
#define UTILITY_MATH_TRANSFORM_H

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "utility/math/angle.h"

/**
 * Matrix related helper methods
 *
 * @author Brendan Annable
 */

namespace utility {
namespace math {
    namespace transform {

        template <typename Scalar>
        inline Scalar angle(const Eigen::Transform<Scalar, 2, Eigen::Affine>& t) {
            // .smallestAngle() returns in the range -pi to pi, so this is a "normalised" angle
            return Eigen::Rotation2D<Scalar>(t.linear()).smallestAngle();
        }

        template <typename Scalar, int dim>
        inline Eigen::Transform<Scalar, dim, Eigen::Affine> worldToLocal(const Eigen::Transform<Scalar, dim, Eigen::Affine>& world, const Eigen::Transform<Scalar, dim, Eigen::Affine>& reference) {
            // http://en.wikipedia.org/wiki/Change_of_basis
            return reference.inverse() * world;
        }

        template <typename Scalar, int dim>
        inline Eigen::Transform<Scalar, dim, Eigen::Affine> localToWorld(const Eigen::Transform<Scalar, dim, Eigen::Affine>& world, const Eigen::Transform<Scalar, dim, Eigen::Affine>& reference) {
            // http://en.wikipedia.org/wiki/Change_of_basis
            return reference * world;
        }

        template <typename Scalar>
        inline Eigen::Transform<Scalar, 2, Eigen::Affine> interpolate(const Eigen::Transform<Scalar, 2, Eigen::Affine>& current, const Eigen::Transform<Scalar, 2, Eigen::Affine>& target, double t) {
            Eigen::Transform<Scalar, 2, Eigen::Affine> result;
            // current * CreateRotation(t * ( current(theta) - target(theta) ))
            result.linear() = current.linear() * Eigen::Rotation2D<Scalar>(t * angle(current) - angle(target)).toRotationMatrix();
            result.translation() = current.translation() + t * (target.translation() - current.translation());
            return result;
        }

        template <typename Scalar>
        inline Eigen::Transform<Scalar, 3, Eigen::Affine> interpolate(const Eigen::Transform<Scalar, 3, Eigen::Affine>& t1, Eigen::Transform<Scalar, 3, Eigen::Affine>& t2, Scalar alpha) {
            
            // Create quaternions from the transforms' rotation matrices
            Eigen::Quaternion t1Rot = Eigen::Quaternion(t1.linear());
            Eigen::Quaternion t2Rot = Eigen::Quaternion(t2.linear());

            // Extract the translation vectors
            Eigen::Matrix<Scalar, 3, 1> t1Translation = t1.translation();
            Eigen::Matrix<Scalar, 3, 1> t2Tranlsation = t2.translation();

            // Create and return interpolated transform
            Eigen::Transform<Scalar, 3, Eigen::Affine> result;
            result.linear() = (t1Rot.slerp(alpha, t2Rot)).toRotationMatrix();
            result.translation() = alpha * (t2Tranlsation - t1Translation);
            return result;
        }

        template <typename Scalar>
        inline Eigen::Transform<Scalar, 3, Eigen::Affine> rotateLocal(const Eigen::Transform<Scalar, 3, Eigen::Affine>& world, const Eigen::AngleAxis<Scalar>& rotation, const Eigen::Transform<Scalar, 3, Eigen::Affine>& local) {
            
            // Create a transform with zero translation and .linear() = rotation param
            Eigen::Transform<Scalar, 3, Eigen::Affine> temp;
            temp.translation() = Eigen::Matrix<Scalar, 3, 1>::Zero();
            temp.linear() = rotation.toRotationMatrix();
            return localToWorld(temp*worldToLocal(world, local), local);
        }

        template <typename Scalar>
        inline Eigen::Transform<Scalar, 3, Eigen::Affine> rotateXLocal(const Eigen::Transform<Scalar, 3, Eigen::Affine>& world, const Scalar radians, const Eigen::Transform<Scalar, 3, Eigen::Affine>& local) {
            return localToWorld((Eigen::AngleAxis<Scalar>(radians, Eigen::Matrix<Scalar, 3, 1>::UnitX()).toRotationMatrix() * worldToLocal(world, local)), local);
        }

        template <typename Scalar>
        inline Eigen::Transform<Scalar, 3, Eigen::Affine> rotateYLocal(const Eigen::Transform<Scalar, 3, Eigen::Affine>& world, const Scalar radians, const Eigen::Transform<Scalar, 3, Eigen::Affine>& local) {
            return localToWorld((Eigen::AngleAxis<Scalar>(radians, Eigen::Matrix<Scalar, 3, 1>::UnitY()).toRotationMatrix() * worldToLocal(world, local)), local);
        }

        template <typename Scalar>
        inline Eigen::Transform<Scalar, 3, Eigen::Affine> rotateZLocal(const Eigen::Transform<Scalar, 3, Eigen::Affine>& world, const Scalar radians, const Eigen::Transform<Scalar, 3, Eigen::Affine>& local) {
            return localToWorld((Eigen::AngleAxis<Scalar>(radians, Eigen::Matrix<Scalar, 3, 1>::UnitZ()).toRotationMatrix() * worldToLocal(world, local)), local);
        }

        template <typename Scalar>
        inline Eigen::Transform<Scalar, 3, Eigen::Affine> rotateX(const Scalar radians, const Eigen::Transform<Scalar, 3, Eigen::Affine>& t) {
            Eigen::Transform<Scalar, 3, Eigen::Affine> rotationMatrix;
            rotationMatrix.translation() = Eigen::Matrix<Scalar, 3, 1>::Zero();
            rotationMatrix.linear() = Eigen::AngleAxis<Scalar>(radians, Eigen::Matrix<Scalar, 3, 1>::UnitX()).toRotationMatrix();
            return t * rotationMatrix;
        }

 
        template <typename Scalar>
        inline Eigen::Transform<Scalar, 3, Eigen::Affine> rotateY(const Scalar radians, const Eigen::Transform<Scalar, 3, Eigen::Affine>& t) {
            Eigen::Transform<Scalar, 3, Eigen::Affine> rotationMatrix;
            rotationMatrix.translation() = Eigen::Matrix<Scalar, 3, 1>::Zero();
            rotationMatrix.linear() = Eigen::AngleAxis<Scalar>(radians, Eigen::Matrix<Scalar, 3, 1>::UnitY()).toRotationMatrix();
            return t * rotationMatrix;
        }

 
        template <typename Scalar>
        inline Eigen::Transform<Scalar, 3, Eigen::Affine> rotateZ(const Scalar radians, const Eigen::Transform<Scalar, 3, Eigen::Affine>& t) {
            Eigen::Transform<Scalar, 3, Eigen::Affine> rotationMatrix;
            rotationMatrix.translation() = Eigen::Matrix<Scalar, 3, 1>::Zero();
            rotationMatrix.linear() = Eigen::AngleAxis<Scalar>(radians, Eigen::Matrix<Scalar, 3, 1>::UnitZ()).toRotationMatrix();
            return t * rotationMatrix;
        }

        template <typename Scalar>
        inline Eigen::Transform<Scalar, 3, Eigen::Affine> createAffine(const Scalar x, const Scalar y, const Scalar z, const Scalar xRad, const Scalar yRad, const Scalar zRad) {
            Eigen::Transform<Scalar, 3, Eigen::Affine> result;
            result.translation() = Eigen::Matrix<Scalar, 3, 1>({x, y, z});
            result.linear().matrix() = Eigen::Matrix<Scalar, 3, 3>::Identity();
            result.rotate(Eigen::AngleAxis<Scalar>(zRad, Eigen::Matrix<Scalar, 3, 1>::UnitZ()));
            result.rotate(Eigen::AngleAxis<Scalar>(yRad, Eigen::Matrix<Scalar, 3, 1>::UnitY()));
            result.rotate(Eigen::AngleAxis<Scalar>(xRad, Eigen::Matrix<Scalar, 3, 1>::UnitX()));
            return result;
        }

        template <typename Scalar>
        inline Eigen::Transform<Scalar, 3, Eigen::Affine> twoD_to_threeD(const Eigen::Transform<Scalar, 2, Eigen::Affine>& t) {
            Eigen::Transform<Scalar, 3, Eigen::Affine> result;
            result = rotateZ(angle(t), Eigen::Transform<Scalar, 3, Eigen::Affine>::Identity());
            result.translation() = Eigen::Vector3d({t.translation()[0], t.translation()[1], 0});
            return result;
        }

    }  // namespace transform
}  // namespace math
}  // namespace utility
#endif  // UTILITY_MATH_TRANSFORM_H
