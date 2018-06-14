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

#include "RansacCircleModel.h"

namespace utility {
namespace math {
    namespace ransac {

        bool RansacCircleModel::regenerate(const std::array<DataPoint, REQUIRED_POINTS>& points) {
            if (points.size() == REQUIRED_POINTS && !arma::all(points[0] == points[1])
                && !arma::all(points[0] == points[2]) && !arma::all(points[1] == points[2])) {
                return setFromPoints(points[0], points[1], points[2], 1.0e-2);
            }

            else {
                return false;
            }
        }

        double RansacCircleModel::calculateError(const DataPoint& p) const {
            double error = distanceToPoint(p);
            return error * error;
        }
    }  // namespace ransac
}  // namespace math
}  // namespace utility
