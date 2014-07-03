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

#include <armadillo>

namespace utility {
namespace math {
namespace geometry {

    class Circle {
    public:
        double radius;
        arma::vec2 centre;

        Circle();

        Circle(const double& radius, const arma::vec2& centre);

        Circle(const arma::vec2& a, const arma::vec2& b, const arma::vec2& c, const double tolerance = std::numeric_limits<double>::min());

        bool setFromPoints(const arma::vec2& a, const arma::vec2& b, const arma::vec2& c, const double tolerance = std::numeric_limits<double>::min());

        double distanceToPoint(const arma::vec2& point) const;

        arma::vec2 orthogonalProjection(const arma::vec2& x);

        //Perform a least squares fit on a line, optionally using a distance
        //squared threshold away from the current model to filter candidates
        template <typename Iterator>
        void leastSquaresUpdate(Iterator& first, Iterator& last, const double& candidateThreshold = std::numeric_limits<double>::max()) {

            //Perform a least squares fit on a circle, optionally using a distance
            //squared threshold away from the current model to filter candidates

            //Method posted on a mailing list at:
            //http://www.math.niu.edu/~rusin/known-math/96/circle.fit
            //Reference: [Pawel Gora, Zdislav V. Kovarik, Daniel Pfenniger, Condensed by Amara Graps]
            arma::mat linearEq1(std::distance(first, last), 3);
            arma::vec linearEq2(std::distance(first, last));
            size_t i = 0;
            for (auto it = first; it != last; ++it) {
                const double diff = distanceToPoint(*it);
                if (diff * diff < candidateThreshold) {
                    linearEq1.row(i).cols(0,1) = (*it).t();
                    linearEq1(i, 2) = 1.0;
                    linearEq2(i) = -arma::dot((*it),(*it));
                    ++i;
                }
            }
            linearEq1.shed_rows(i, std::distance(first, last) - 1);
            linearEq2.shed_rows(i, std::distance(first, last) - 1);
            arma::vec3 results = arma::solve(linearEq1, linearEq2);
            centre = { results[0] * 0.5, results[1] * 0.5 };
            radius = arma::dot(centre, centre) - results[2];
        }
    };

}
}
}

#endif
