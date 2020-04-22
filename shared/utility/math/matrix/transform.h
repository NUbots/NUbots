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

        inline Eigen::Affine2d worldToLocal(const Eigen::Affine2d world, const Eigen::Affine2d& reference) {
            Eigen::Affine2d diff;
            double angle       = utility::math::angle::normalizeAngle(Eigen::Rotation2Dd(reference.linear()).angle()
                                                                - Eigen::Rotation2Dd(world.linear()).angle());
            diff.linear()      = Eigen::Rotation2Dd(angle).toRotationMatrix();
            diff.translation() = reference.translation() - world.translation();

            Eigen::Affine2d result;
            result.translation() = world.linear() * diff.translation();
            result.linear()      = diff.linear();
            return result;
        }

        template <typename Scalar>
        inline  Eigen::Transform<Scalar, 2, Eigen::Affine> projectTo2D(const Eigen::Transform<Scalar, 3, Eigen::Affine> t,
                                                                        const Eigen::Matrix<Scalar, 3, 1>& yawAxis     = Eigen::Matrix<Scalar, 3, 1>(0, 0, 1),
                                                                        const Eigen::Matrix<Scalar, 3, 1>& forwardAxis = Eigen::Matrix<Scalar, 3, 1>(1, 0, 0)) const {
            Eigen::Transform<Scalar, 2, Eigen::Affine> result;

            // Translation
            Eigen::Matrix<Scalar, 3, 1> orthoForwardAxis = yawAxis.cross(forwardAxis.cross(yawAxis)).normalize();
            Eigen::Matrix<Scalar, 3, 1> r                = t.translation();

            // Create a rotation matrix, then assign the columns of it separately
            Eigen::Matrix<Scalar, 3, 3> newSpaceToWorld(Eigen::Matrix<Scalar, 3, 3>::Identity());
            newSpaceToWorld.block<2, 0>(0, 0)        = orthoForwardAxis;
            newSpaceToWorld.block<2, 1>(0, 1)        = yawAxis.cross(orthoForwardAxis);
            newSpaceToWorld.block<2, 2>(0, 2)        = yawAxis;

            Eigen::Matrix<Scalar, 3, 3> worldToNewSpace(newSpaceToWorld.inverse());

            Eigen::Matrix<Scalar, 3, 1> rNewSpace       = worldToNewSpace * r;
            result.translation().x()                = rNewSpace.x();
            result.translation().y()                = rNewSpace.y();

            // Rotation
            Eigen::Rotation<Scalar, 3> rot       = t.rotation();
            Eigen::Matrix<Scalar, 3, 1> x         = rot.block<2, 0>(0, 0);
            Eigen::Matrix<Scalar, 3, 1> xNew      = worldToNewSpace * x;

            result.linear()       = Eigen::Rotation2D<Scalar>(std::atan2(xNew.x(), xNew.y())).toRotationMatrix();

            // std::cerr << "in = \n" << *this << std::endl;
            // std::cerr << "out = \n" << result << std::endl;
            return result;
        }

    }  // namespace transform
}  // namespace math
}  // namespace utility
#endif  // UTILITY_MATH_TRANSFORM_H
