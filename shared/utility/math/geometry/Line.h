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


#ifndef UTILITY_MATH_LINE_H
#define UTILITY_MATH_LINE_H

#include <armadillo>
#include <iostream>
#include <vector>
#include <limits>

namespace utility {
namespace math {
namespace geometry {

    class Line {
    private:
        arma::vec2 norm;
        double distance;

    public:
        Line();

        Line(const arma::vec2& a, const arma::vec2& b);

        void setFromPoints(const arma::vec2& a, const arma::vec2& b);

        double x(const double& y);
        double y(const double& x);
    };

}
}
}
#endif
