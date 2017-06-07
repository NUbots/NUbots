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

#ifndef UTILITY_MATH_GEOMETRY_ROTATEDRECTANGLE_H
#define UTILITY_MATH_GEOMETRY_ROTATEDRECTANGLE_H

#include <ostream>
#include <vector>

#include "utility/math/matrix/Transform2D.h"

namespace utility {
namespace math {
namespace geometry {

    using utility::math::matrix::Transform2D;

    class RotatedRectangle {
    private:
        Transform2D transform;
        Eigen::Vector2d size;

    public:
        RotatedRectangle(const Transform2D& trans, const Eigen::Vector2d& size);

        Transform2D getTransform() const;
        Eigen::Vector2d getSize()       const;
        Eigen::Vector2d getPosition()   const;
        double getRotation()       const;
    };
}
}
}

#endif
