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

#ifndef UTILITY_MATH_GEOMETRY_ROTATEDRECTANGLE_HPP
#define UTILITY_MATH_GEOMETRY_ROTATEDRECTANGLE_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <ostream>
#include <vector>

namespace utility::math::geometry {

    template <typename Scalar>
    class RotatedRectangle {
        using AffineType   = Eigen::Transform<Scalar, 2, Eigen::Affine>;
        using VectorType   = Eigen::Matrix<Scalar, 2, 1>;
        using RotationType = Eigen::Rotation2D<Scalar>;

    private:
        AffineType transform;
        VectorType size;

    public:
        RotatedRectangle(const AffineType& trans, const VectorType& size) : transform(transform), size(size) {}

        AffineType getTransform() const {
            return transform;
        }
        VectorType getSize() const {
            return size;
        }
        VectorType getPosition() const {
            return transform.translation();
        }
        Scalar getRotationAngle() const {
            return getRotation().smallestAngle();
        }
        RotationType getRotation() const {
            return RotationType(transform.rotation());
        }
    };
}  // namespace utility::math::geometry

#endif  // UTILITY_MATH_GEOMETRY_ROTATEDRECTANGLE_HPP
