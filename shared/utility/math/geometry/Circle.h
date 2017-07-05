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

#ifndef UTILITY_MATH_GEOMETRY_CIRCLE_H
#define UTILITY_MATH_GEOMETRY_CIRCLE_H

#include <Eigen/Core>
#include <Eigen/QR>

#include "utility/math/comparison.h"

namespace utility {
namespace math {
    namespace geometry {

        class Circle {
        public:
            double radius;
            double radiusSq;
            Eigen::Vector2d centre;

            Circle();

            Circle(const double& radius, const Eigen::Vector2d& centre);

            Circle(const Eigen::Vector2d& a,
                   const Eigen::Vector2d& b,
                   const Eigen::Vector2d& c,
                   const double tolerance = std::numeric_limits<double>::min());

            bool setFromPoints(const Eigen::Vector2d& a,
                               const Eigen::Vector2d& b,
                               const Eigen::Vector2d& c,
                               const double tolerance = std::numeric_limits<double>::min());

            double distanceToPoint(const Eigen::Vector2d& point) const;

            double squaresDifference(const Eigen::Vector2d& point) const;

            Eigen::Vector2d orthogonalProjection(const Eigen::Vector2d& x) const;

            // Perform a least squares fit on a line, optionally using a distance
            // squared threshold away from the current model to filter candidates
            template <typename Iterator>
            void leastSquaresUpdate(Iterator& first,
                                    Iterator& last,
                                    const double& candidateThreshold = std::numeric_limits<double>::max()) {

                // Perform a least squares fit on a circle, optionally using a distance
                // squared threshold away from the current model to filter candidates

                // Method posted on a mailing list at:
                // http://www.math.niu.edu/~rusin/known-math/96/circle.fit
                // Reference: [Pawel Gora, Zdislav V. Kovarik, Daniel Pfenniger, Condensed by Amara Graps]
                Eigen::Matrix<double, Eigen::Dynamic, 3> linearEq1(std::distance(first, last), 3);
                Eigen::VectorXd linearEq2(std::distance(first, last));
                uint i = 0;

                for (auto it = first; it != last; ++it) {
                    const double diff = distanceToPoint(*it);
                    if (diff * diff < candidateThreshold) {
                        linearEq1.row(i).head<2>() = (*it).transpose();
                        linearEq1(i, 2) = 1.0;
                        linearEq2(i) = -(*it).dot((*it));
                        ++i;
                    }
                }

                if (i != 0) {
                    Eigen::ColPivHouseholderQR<Eigen::Matrix<double, Eigen::Dynamic, 3>> solver(linearEq1.topRows(i));
                    Eigen::Vector3d results = solver.solve(linearEq2.head(i));
                    centre << std::abs(results[0] * 0.5) * utility::math::sgn(centre[0]),
                        std::abs(results[1] * 0.5) * utility::math::sgn(centre[1]);
                    radiusSq = centre.dot(centre) - results[2];
                    radius   = std::sqrt(radiusSq);
                }
            }

            Eigen::Vector2d getEdgePoints(uint y) const;
            Eigen::Vector2d getEdgePoints(double y) const;
        };
    }
}
}

#endif
