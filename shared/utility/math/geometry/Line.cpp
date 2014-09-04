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
        setFromPoints(std::forward<const arma::vec2&>(a), std::forward<const arma::vec2&>(b));
    }

    void Line::setFromPoints(const arma::vec2& a, const arma::vec2& b) {
        arma::vec2 l = arma::normalise(a - b);

        normal = arma::vec2({ -l[1], l[0] });
        distance = arma::dot(normal, a);
    }

    double Line::x(const double& y) const {
        return (distance - y * normal[1]) / normal[0];
    }

    double Line::y(const double& x) const {
        return (distance - x * normal[0]) / normal[1];
    }

    double Line::distanceToPoint(const arma::vec2& point) const {
        return arma::dot(point, normal) - distance;
    }

    double Line::tangentialDistanceToPoint(const arma::vec2& x) const {
        return arma::dot(arma::vec2({ -normal[1], normal[0] }), x);
    }

    arma::vec2 Line::pointFromTangentialDistance(const double& x) const {
        return normal * distance + arma::vec2({ -normal[1], normal[0] }) * x;
    }

    bool Line::isHorizontal() const {
        return normal[0] == 0;
    }

    bool Line::isVertical() const {
        return normal[1] == 0;
    }

    arma::vec2 Line::orthogonalProjection(const arma::vec2& x) const {
        return x - (arma::dot(x, normal) - distance) * normal;
    }


    Line Line::getParallelLineThrough(const arma::vec2& x) const {
        Line result;
        result.normal = normal;
        result.distance = arma::dot(x, normal);
        return result;
    }

}
}
}

