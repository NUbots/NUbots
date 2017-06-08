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

    Line::Line() : normal(arma::fill::zeros), distance(0.0) {
    }

    Line::Line(const Eigen::Vector2d& n, const double& d) : normal(n), distance(d) {
    }

    Line::Line(const Eigen::Vector2d& a, const Eigen::Vector2d& b) : normal(arma::fill::zeros), distance(0.0) {
        setFromPoints(std::forward<const Eigen::Vector2d&>(a), std::forward<const Eigen::Vector2d&>(b));
    }

    void Line::setFromPoints(const Eigen::Vector2d& a, const Eigen::Vector2d& b) {
        Eigen::Vector2d l = (a - b).normalize();

        normal = Eigen::Vector2d( -l[1], l[0] );
        distance = normal.dot(a);
    }

    double Line::x(const double& y) const {
        return (distance - y * normal[1]) / normal[0];
    }

    double Line::y(const double& x) const {
        return (distance - x * normal[0]) / normal[1];
    }

    double Line::distanceToPoint(const Eigen::Vector2d& point) const {
        return point.dot(normal) - distance;
    }

    double Line::tangentialDistanceToPoint(const Eigen::Vector2d& x) const {
        return tangent().dot(x);
    }

    Eigen::Vector2d Line::pointFromTangentialDistance(const double& x) const {
        return normal * distance + tangent() * x;
    }

    bool Line::isHorizontal() const {
        return normal[0] == 0;
    }

    bool Line::isVertical() const {
        return normal[1] == 0;
    }

    Eigen::Vector2d Line::orthogonalProjection(const Eigen::Vector2d& x) const {
        return x - (x.dot(normal) - distance) * normal;
    }


    Line Line::getParallelLineThrough(const Eigen::Vector2d& x) const {
        Line result;
        result.normal = normal;
        result.distance = x.dot(normal);
        return result;
    }

    Eigen::Vector2d Line::tangent() const {
        return {-normal[1], normal[0]};
    }

    Eigen::Vector2d Line::intersect(const Line& line) const {

        Eigen::Vector2d direction1 = tangent();
        Eigen::Vector2d direction2 = line.tangent();
        Eigen::Vector2d point1 = pointFromTangentialDistance(0);
        Eigen::Vector2d point2 = line.pointFromTangentialDistance(0);

        //Setup linear equations:
        arma::mat Ainverse;
        //Check extended lines intersect at all
        double determinant = - direction1[0] * direction2[1] + direction1[1] * direction2[0];
        if (determinant == 0) {
            throw std::domain_error("Line::intersect - Lines do not intersect (parallel)");
        } else {
            Ainverse << -direction2[1] << direction2[0] << arma::endr
                     << -direction1[1] << direction1[0];
            Ainverse *= 1 / determinant;
        }

        arma::vec/*2*/ tValues = Ainverse * (arma::vec(point2) - arma::vec(point1));  //arma::meat

        return point1 + tValues[0] * direction1;
    }

}
}
}

