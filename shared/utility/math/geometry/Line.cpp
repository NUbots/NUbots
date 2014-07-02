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

#include "Line.h"

namespace utility {
namespace math {
namespace geometry {

    Line::Line() {
    }

    Line::Line(const arma::vec2& a, const arma::vec2& b) {
        setFromPoints(std::forward<const arma::vec2&>(a), std::forward<const arma::vec2&>(b))
    }

    void Line::setFromPoints(const arma::vec2& a, const arma::vec2& b) {
        auto l = arma::norm(a - b);

        normal = arma::vec2({ -l[1], l[0] });
        distance = arma::dot(normal, a);
    }

    double Line::x(double y) {
        return (d - y * n[1]) / n[0];
    }

    double Line::y(double x) {
        return (d - x * n[0]) / n[1];
    }


    double Line::distanceToPoint(const arma::vec2& point) {
        return arma::dot(point, normal) - distance;
    }
}
}
}

