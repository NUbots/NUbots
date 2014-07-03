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

#include "RansacCircleModel.h"

namespace utility {
namespace math {
namespace ransac {

  bool RansacCircleModel::regenerate(const std::vector<DataPoint>& points) {
        if (points.size() == REQUIRED_POINTS
            && !arma::all(points[0] == points[1])
            && !arma::all(points[0] == points[2])
            && !arma::all(points[1] == points[2])) {
            return constructFromPoints(points[0], points[1], points[2], 1.0e-2);
        }

        else {
            return false;
        }
    }

    double RansacCircleModel::calculateError(const DataPoint& p) const {
        double error = arma::norm(p - centre) - radius;
        return error * error;
    }

    double RansacCircleModel::getRadius() const {
        return radius;
    }

    RansacCircleModel::DataPoint RansacCircleModel::getCentre() const {
        return centre;
    }

    bool RansacCircleModel::constructFromPoints(const DataPoint& point1, const DataPoint& point2, const DataPoint& point3, double tolerance) {
        DataPoint ab = point1 - point2;
        DataPoint bc = point2 - point3;
        double det = ((ab[0] * bc[1]) - (bc[0] * ab[1]));

        if (std::abs(det) < tolerance) {
            return false;
        }

        // double b_len_sqr = p2.squareAbs();
        double b_len_sqr = arma::dot(point2, point2);

        double ab_norm = (arma::dot(point1, point1) - b_len_sqr) / 2.0;
        double bc_norm = (b_len_sqr - arma::dot(point3, point3)) / 2.0;

        det = 1 / det;
        centre[0] = ((ab_norm * bc[1]) - (bc_norm * ab[1])) * det;
        centre[1] = ((ab[0] * bc_norm) - (bc[0] * ab_norm)) * det;

        radius = arma::norm(centre - point1, 2);

        return true;
    }

}
}
}