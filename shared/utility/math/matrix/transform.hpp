/*
 * MIT License
 *
 * Copyright (c) 2020 NUbots
 *
 * This file is part of the NUbots codebase.
 * See https://github.com/NUbots/NUbots for further info.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef UTILITY_MATH_TRANSFORM_HPP
#define UTILITY_MATH_TRANSFORM_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "utility/math/angle.hpp"

/**
 * Matrix related helper methods
 *
 * @author Brendan Annable
 */

namespace utility::math::transform {

    template <typename Scalar>
    inline Scalar angle(const Eigen::Transform<Scalar, 2, Eigen::Isometry>& t) {
        // .smallestAngle() returns in the range -pi to pi, so this is a "normalised" angle
        return Eigen::Rotation2D<Scalar>(t.linear()).smallestAngle();
    }

    template <typename Scalar, int dim>
    inline Eigen::Transform<Scalar, dim, Eigen::Isometry> worldToLocal(
        const Eigen::Transform<Scalar, dim, Eigen::Isometry>& world,
        const Eigen::Transform<Scalar, dim, Eigen::Isometry>& reference) {
        // http://en.wikipedia.org/wiki/Change_of_basis
        return reference.inverse() * world;
    }

    template <typename Scalar, int dim>
    inline Eigen::Transform<Scalar, dim, Eigen::Isometry> localToWorld(
        const Eigen::Transform<Scalar, dim, Eigen::Isometry>& world,
        const Eigen::Transform<Scalar, dim, Eigen::Isometry>& reference) {
        // http://en.wikipedia.org/wiki/Change_of_basis
        return reference * world;
    }

    template <typename Scalar>
    inline Eigen::Transform<Scalar, 2, Eigen::Isometry> interpolate(
        const Eigen::Transform<Scalar, 2, Eigen::Isometry>& current,
        const Eigen::Transform<Scalar, 2, Eigen::Isometry>& target,
        const Scalar& t) {
        Eigen::Transform<Scalar, 2, Eigen::Isometry> result;
        // result = current * CreateRotation(t * ( current(theta) - target(theta) ))
        result.linear() =
            current.rotation() * Eigen::Rotation2D<Scalar>(t * (angle(current) - angle(target))).toRotationMatrix();
        result.translation() = current.translation() + t * (target.translation() - current.translation());
        return result;
    }

    template <typename Scalar>
    inline Eigen::Transform<Scalar, 3, Eigen::Isometry> interpolate(
        const Eigen::Transform<Scalar, 3, Eigen::Isometry>& t1,
        const Eigen::Transform<Scalar, 3, Eigen::Isometry>& t2,
        const Scalar& alpha) {

        // Create quaternions from the transforms' rotation matrices
        Eigen::Quaternion<Scalar> t1Rot = Eigen::Quaternion<Scalar>(t1.rotation());
        Eigen::Quaternion<Scalar> t2Rot = Eigen::Quaternion<Scalar>(t2.rotation());

        // Extract the translation vectors
        Eigen::Matrix<Scalar, 3, 1> t1Translation = t1.translation();
        Eigen::Matrix<Scalar, 3, 1> t2Translation = t2.translation();

        // Create and return interpolated transform
        Eigen::Transform<Scalar, 3, Eigen::Isometry> result;
        result.linear()      = (t1Rot.slerp(alpha, t2Rot)).toRotationMatrix();
        result.translation() = alpha * (t2Translation - t1Translation) + t1Translation;
        return result;
    }

    template <typename Scalar>
    inline Eigen::Transform<Scalar, 3, Eigen::Isometry> rotateLocal(
        const Eigen::Transform<Scalar, 3, Eigen::Isometry>& world,
        const Eigen::AngleAxis<Scalar>& rotation,
        const Eigen::Transform<Scalar, 3, Eigen::Isometry>& local) {

        // Create a transform with zero translation and .linear() = rotation parameter
        // Then use it to rotate the innermost transform
        Eigen::Transform<Scalar, 3, Eigen::Isometry> temp = Eigen::Transform<Scalar, 3, Eigen::Isometry>::Identity();
        temp.linear()                                     = rotation.toRotationMatrix();
        return localToWorld(temp * worldToLocal(world, local), local);
    }

    template <typename Scalar>
    inline Eigen::Transform<Scalar, 3, Eigen::Isometry> rotateXLocal(
        const Eigen::Transform<Scalar, 3, Eigen::Isometry>& world,
        const Scalar& radians,
        const Eigen::Transform<Scalar, 3, Eigen::Isometry>& local) {
        // Do the local rotation with an AngleAxis on the UnitX vector
        return localToWorld((Eigen::AngleAxis<Scalar>(radians, Eigen::Matrix<Scalar, 3, 1>::UnitX()).toRotationMatrix()
                             * worldToLocal(world, local)),
                            local);
    }

    template <typename Scalar>
    inline Eigen::Transform<Scalar, 3, Eigen::Isometry> rotateYLocal(
        const Eigen::Transform<Scalar, 3, Eigen::Isometry>& world,
        const Scalar& radians,
        const Eigen::Transform<Scalar, 3, Eigen::Isometry>& local) {
        // Do the local rotation with an AngleAxis on the UnitY vector
        return localToWorld((Eigen::AngleAxis<Scalar>(radians, Eigen::Matrix<Scalar, 3, 1>::UnitY()).toRotationMatrix()
                             * worldToLocal(world, local)),
                            local);
    }

    template <typename Scalar>
    inline Eigen::Transform<Scalar, 3, Eigen::Isometry> rotateZLocal(
        const Eigen::Transform<Scalar, 3, Eigen::Isometry>& world,
        const Scalar& radians,
        const Eigen::Transform<Scalar, 3, Eigen::Isometry>& local) {
        // Do the local rotation with an AngleAxis on the UnitZ vector
        return localToWorld((Eigen::AngleAxis<Scalar>(radians, Eigen::Matrix<Scalar, 3, 1>::UnitZ()).toRotationMatrix()
                             * worldToLocal(world, local)),
                            local);
    }

    template <typename Scalar>
    inline Eigen::Transform<Scalar, 3, Eigen::Isometry> rotateX(const Scalar& radians,
                                                                const Eigen::Transform<Scalar, 3, Eigen::Isometry>& t) {
        Eigen::Transform<Scalar, 3, Eigen::Isometry> rotationMatrix =
            Eigen::Transform<Scalar, 3, Eigen::Isometry>::Identity();
        rotationMatrix.linear() =
            Eigen::AngleAxis<Scalar>(radians, Eigen::Matrix<Scalar, 3, 1>::UnitX()).toRotationMatrix();
        return t * rotationMatrix;
    }


    template <typename Scalar>
    inline Eigen::Transform<Scalar, 3, Eigen::Isometry> rotateY(const Scalar& radians,
                                                                const Eigen::Transform<Scalar, 3, Eigen::Isometry>& t) {
        Eigen::Transform<Scalar, 3, Eigen::Isometry> rotationMatrix =
            Eigen::Transform<Scalar, 3, Eigen::Isometry>::Identity();
        rotationMatrix.linear() =
            Eigen::AngleAxis<Scalar>(radians, Eigen::Matrix<Scalar, 3, 1>::UnitY()).toRotationMatrix();
        return t * rotationMatrix;
    }


    template <typename Scalar>
    inline Eigen::Transform<Scalar, 3, Eigen::Isometry> rotateZ(const Scalar& radians,
                                                                const Eigen::Transform<Scalar, 3, Eigen::Isometry>& t) {
        Eigen::Transform<Scalar, 3, Eigen::Isometry> rotationMatrix;
        rotationMatrix.translation() = Eigen::Matrix<Scalar, 3, 1>::Zero();
        rotationMatrix.linear() =
            Eigen::AngleAxis<Scalar>(radians, Eigen::Matrix<Scalar, 3, 1>::UnitZ()).toRotationMatrix();
        return t * rotationMatrix;
    }

    template <typename Scalar>
    inline Eigen::Transform<Scalar, 3, Eigen::Isometry> twoD_to_threeD(
        const Eigen::Transform<Scalar, 2, Eigen::Isometry>& t) {
        Eigen::Transform<Scalar, 3, Eigen::Isometry> result;

        // Rotate an identity transform on the Z axis the angle of the passed in 2D transform
        result               = rotateZ(angle(t), Eigen::Transform<Scalar, 3, Eigen::Isometry>::Identity());
        result.translation() = Eigen::Vector3d(t.translation().x(), t.translation().y(), 0);
        return result;
    }

    inline Eigen::Isometry2d lookAt(const Eigen::Vector2d& from, const Eigen::Vector2d& to) {
        Eigen::Isometry2d result;
        result.translation() = from;
        result.linear()      = Eigen::Rotation2Dd(utility::math::angle::vectorToBearing(to - from)).toRotationMatrix();
        return result;
    }
}  // namespace utility::math::transform
#endif  // UTILITY_MATH_TRANSFORM_HPP
