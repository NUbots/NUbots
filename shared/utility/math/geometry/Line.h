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


#ifndef UTILITY_MATH_GEOMETRY_LINE_H
#define UTILITY_MATH_GEOMETRY_LINE_H

#include <armadillo>
#include <iostream>
#include <vector>
#include <limits>

namespace utility {
namespace math {
namespace geometry {

class Line {
    private:
        arma::vec2 normal;
        double distance;

    public:
        Line();

        Line(const arma::vec2& a, const arma::vec2& b);

        void setFromPoints(const arma::vec2& a, const arma::vec2& b);

        double x(const double& y) const;
        double y(const double& x) const;

        double distanceToPoint(const arma::vec2& point) const;

        bool isHorizontal() const;
        bool isVertical() const;

        arma::vec2 orthogonalProjection(const arma::vec2 x);

        //Perform a least squares fit on a line, optionally using a distance
        //squared threshold away from the current model to filter candidates
        template <typename Iterator>
        void leastSquaresUpdate(const Iterator& first,
                                const Iterator& last,
                                const double& candidateThreshold = std::numeric_limits<double>::max()) {

            arma::vec2 average({ 0.0, 0.0 });
            arma::vec2 squaredaverage({ 0.0, 0.0 });
            double jointaverage = 0.0;

            //step 1: calculate means and grab candidates
            for (auto it = first; it != last; ++it) {
                const double diff = distanceToPoint(*it);
                if ( diff*diff < candidateThreshold ) {
                    average += *it;
                    squaredaverage += (*it) % (*it);
                    jointaverage += (*it)[0] * (*it)[1];
                }
            }

            //step 2: calculate the slope and intercept - this is long because we need 2 cases for this line representation
            arma::vec2 line;
            double m,b;
            if (normal[0] > normal[1]) { //check whether to use y=mx+b or x=my+b
                m = (jointaverage - average[0] * average[1])/(squaredaverage[0]);
                b = average[1] - m * average[0];
                line = arma::normalise(arma::vec2({ 1.0, m }));

            } else {
                m = (jointaverage - average[0] * average[1]) / (squaredaverage[1]);
                b = average[0] - m*average[1];
                line = arma::normalise(arma::vec2({ m, 1.0 }));
            }

            normal = { -line[1], line[0] };
            distance = m * b;
        }
    };

}
}
}
#endif
