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

#include "Circle.h"

namespace utility {
namespace math {
namespace geometry {

    Circle::Circle() {
    }

    Circle::Circle(const arma::vec2& a, const arma::vec2& b, , const arma::vec2& c, const double tolerance = std::numeric_limits<double>::min() {
        setFromPoints(std::forward<const arma::vec2&>(a), std::forward<const arma::vec2&>(b), std::forward<const arma::vec2&>(c),tolerance);
    }

    bool Circle::setFromPoints(const arma::vec2& a, const arma::vec2& b, const arma::vec2& c, const double tolerance = std::numeric_limits<double>::min()) {
        const arma::vec2 ab = point1 - point2;
        const arma::vec2 bc = point2 - point3;
        const double det = ((ab[0] * bc[1]) - (bc[0] * ab[1]));

        if (std::abs(det) < tolerance) {
            return false;
        }
        det = 1.0 / det;

        double b_len_sqr = arma::dot(point2, point2);

        double ab_norm = (arma::dot(point1, point1) - b_len_sqr) * 0.5;
        double bc_norm = (b_len_sqr - arma::dot(point3, point3)) * 0.5;

        
        centre[0] = ((ab_norm * bc[1]) - (bc_norm * ab[1])) * det;
        centre[1] = ((ab[0] * bc_norm) - (bc[0] * ab_norm)) * det;

        radius = arma::norm(centre - point1, 2);

        return true;
    }

    double Circle::distanceToPoint(const arma::vec2& point) const {
        return arma::norm(x-centre) - radius;
    }

    arma::vec2 Circle::orthogonalProjection(const arma::vec2 x) {
        return arma::normalise(x - centre)*radius+centre;
    }
    
    void Circle::leastSquaresUpdate(const Iterator<arma::vec2>& first, 
                                  const Iterator<arma::vec2>& last,
                                  const double& candidateThreshold = std::numeric_limits<double>::max()) {
        //Perform a least squares fit on a circle, optionally using a distance
        //squared threshold away from the current model to filter candidates
        
        //Method posted on a mailing list at:
        //http://www.math.niu.edu/~rusin/known-math/96/circle.fit
        //Reference: [Pawel Gora, Zdislav V. Kovarik, Daniel Pfenniger, Condensed by Amara Graps]
        arma::mat linearEq1(3,std::distance(first,last));
        arma::vec linearEq2(std::distance(first,last));
        size_t i = 0;
        for (auto it = first; it != last; ++it) {
            linearEq1.row(i).cols(0,1) = (*it).t();
            linearEq1(i,2) = 1.0;
            linearEq2(i) = -arma::dot((*it),(*it));
            ++i;
        }
        arma::vec3 results = arma::solve(linearEq1,linearEq2);
        centre = {results[0]*0.5, results[1]*0.5};
        radius = arma::dot(centre,centre)-results[2];
    }
}
}
}
