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

    }  // namespace transform
}  // namespace math
}  // namespace utility
#endif  // UTILITY_MATH_TRANSFORM_H
