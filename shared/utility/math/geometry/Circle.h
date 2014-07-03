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

    	
    };
class Line {
    private:
        double radius;
    	arma::vec2 centre;

    public:
        Circle();

        Circle(const arma::vec2& a, const arma::vec2& b, , const arma::vec2& c, const double tolerance = std::numeric_limits<double>::min();

        void setFromPoints(const arma::vec2& a, const arma::vec2& b, const arma::vec2& b, const double tolerance = std::numeric_limits<double>::min();

        double distanceToPoint(const arma::vec2& point) const;
        
        arma::vec2 Line::orthogonalProjection(const arma::vec2 x);
        
        //Perform a least squares fit on a line, optionally using a distance
        //squared threshold away from the current model to filter candidates
        void leastSquaresUpdate(const Iterator<arma::vec2>& first, 
                                const Iterator<arma::vec2>& last,
                                const double& candidateThreshold = std::numeric_limits<double>::max());
    };

}
}
}

#endif
