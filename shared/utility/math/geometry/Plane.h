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
#ifndef UTILITY_MATH_GEOMETRY_PLANE_H
#define UTILITY_MATH_GEOMETRY_PLANE_H

#include <Eigen/Core>

#include "utility/math/geometry/ParametricLine.h"

namespace utility {
namespace math {
    namespace geometry {

        template <int n>
        class Plane {
        private:
            using Vector = Eigen::Matrix<double, n, 1>;

        public:
            Vector normal;
            Vector point;

            Plane() : normal(Vector::Zero()), point(Vector::Zero()) {}
            Plane(Vector normal_, Vector point_) : normal(Vector::Zero()), point(Vector::Zero()) {
                setFromNormal(normal_, point_);
            }

            void setFromNormal(Vector normal_, Vector point_) {
                if (normal_.template lpNorm<1>() <= 0) {
                    throw std::domain_error(
                        "Plane::setFromNormal - Normal is zero vector. Normal to plane must be non-zero!");
                }
                normal = normal_.normalize();
                point  = point_;
            }

            void setFrom3Points(Vector p1, Vector p2, Vector p3) {
                point  = p1;
                normal = (p2 - p1).cross(p3 - p1).normalize();  // Positive if p3 palmside (RHR) relative to p2
                if (normal.template lpNorm<1>() <= 0) {
                    throw std::domain_error("Plane::setFrom3Points - 3 Points are colinear!");
                }
            }

            Vector intersect(ParametricLine<n> l) const {
                double lDotN = l.direction.dot(normal);
                if (lDotN == 0) {
                    throw std::domain_error("Plane::intersect - Plane does not meet line!");
                }
                double tIntersection = (point - l.point).dot(normal) / lDotN;
                if (tIntersection < l.tLimits[0] || tIntersection > l.tLimits[1]) {
                    throw std::domain_error(
                        "Plane::intersect - Plane does not meet line segment (intersection falls off segment)!");
                }
                return tIntersection * l.direction + l.point;
            }
        };
    }
}
}
#endif
