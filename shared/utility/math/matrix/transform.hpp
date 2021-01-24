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

namespace utility {
namespace math {
    namespace transform {

        inline Eigen::Affine2d lookAt(const Eigen::Vector2d& from, const Eigen::Vector2d& to) {
            Eigen::Affine2d result;
            result.translation() = from;
            result.linear() = Eigen::Rotation2Dd(utility::math::angle::vectorToBearing(to - from)).toRotationMatrix();
            return result;
        }

        inline Eigen::Affine2d localToWorld(const Eigen::Affine2d& local, const Eigen::Affine2d& reference) {
            // translates to this + rotZ(this.angle) * reference
            Eigen::Rotation2Dd R(local.linear());
            Eigen::Vector2d newPos = R * reference.translation();
            Eigen::Affine2d result;
            result.translation() = local.translation() + newPos;
            // do not use normalizeAngle here, causes bad things when turning! TODO: unsure on cause
            result.linear() = Eigen::Rotation2Dd(Eigen::Rotation2Dd(local.linear()).angle()
                                                 + Eigen::Rotation2Dd(reference.linear()).angle())
                                  .toRotationMatrix();
            return result;
        }

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

    }  // namespace transform
}  // namespace math
}  // namespace utility
#endif  // UTILITY_MATH_TRANSFORM_HPP
