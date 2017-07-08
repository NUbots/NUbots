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

#ifndef UTILITY_MATH_RANSAC_RANSACCONEMODEL_H
#define UTILITY_MATH_RANSAC_RANSACCONEMODEL_H

#include <array>
#include <armadillo>
#include "utility/math/geometry/Plane.h"

namespace utility {
namespace math {
namespace ransac {

    //3D only!
    class RansacPlaneModel : public utility::math::geometry::Plane<3> {
    public:

        static constexpr size_t REQUIRED_POINTS = 3;
        using DataPoint = arma::vec3;

        bool regenerate(const std::array<DataPoint, REQUIRED_POINTS>& points);

        double calculateError(const DataPoint& p) const;

        template <typename Iterator>
        void refineModel(Iterator&, Iterator&, const double&) {
            //commented out due to performance concerns - works well though
            //leastSquaresUpdate(first,last,candidateThreshold);
        }

        Vector getPoint(float g, float theta)const ;
    };

}
}
}

#endif
