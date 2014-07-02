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

#ifndef UTILITY_MATH_RANSAC_RANSACLINEMODEL_H
#define UTILITY_MATH_RANSAC_RANSACLINEMODEL_H

#include "utility/math/geometry/Line.h"

namespace utility {
namespace math {
namespace ransac {

    template<typename T>
    class RansacLineModel : public utility::math::geometry::Line {
    public:
        static constexpr size_t MIN_POINTS_FOR_FIT = 3;

        RansacLineModel() {}

        bool regenerate(const std::vector<T>& pts) {
            if(pts.size() == minPointsForFit()) {
                setLineFromPoints(pts[0], pts[1]);
                return true;
            }
            else {
                return false;
            }
        }

        double calculateError(const T& p) const {
            double val = getLinePointDistance(std::forward<const T&>(p));
            return val * val;
        }
    };

}
}
}

#endif