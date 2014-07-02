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

#ifndef UTILITY_MATH_RANSAC_RANSACCIRCLEMODEL_H
#define UTILITY_MATH_RANSAC_RANSACCIRCLEMODEL_H

#include <vector>
#include <armadillo>

namespace utility {
namespace math {
namespace ransac {

    template<typename T>
    class RansacCircleModel {
    public:
        static constexpr size_t MIN_POINTS_FOR_FIT = 3;

        RansacCircleModel() : radius(0) {
            // Empty constructor.
        }

        bool regenerate(const std::vector<T>& points) {
            if (points.size() == minPointsForFit()) {
                return constructFromPoints(points[0], points[1], points[2], 1.0e-2);
            }

            else {
                return false;
            }
        }

        double calculateError(const T& p) const {
            double val = arma::norm(p - centre) - radius;
            return val * val;
        }

        double getRadius() const {
            return radius;
        }

        T getCentre() const {
            return centre;
        }

    private:
        bool constructFromPoints(const T& point1, const T& point2, const T& point3, double tolerance = 1.0e-6) {
            T ab = point1 - point2;
            T bc = point2 - point3;
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

    private:
        T centre;
        double radius;
    };

}
}
}

#endif
