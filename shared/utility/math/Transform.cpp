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

#include "Transform.h"

#include <nuclear>

#include "utility/math/angle.h"
#include "utility/math/matrix.h"

namespace utility {
namespace math {

    Transform::Transform() {
        eye(); // identity matrix by default
    }

    Transform& Transform::translate(const arma::vec3& translation) {
        arma::mat44 transform = arma::eye(4,4);
        transform.col(3).rows(0,2) = translation;
        *this *= transform;
        return *this;
    }

    Transform& Transform::translateX(double translation) {
        translate({translation, 0, 0});
        return *this;
    }

    Transform& Transform::translateY(double translation) {
        translate({0, translation, 0});
        return *this;
    }

    Transform& Transform::translateZ(double translation) {
        translate({0, 0, translation});
        return *this;
    }

    Transform& Transform::rotateX(double radians) {
        double c = cos(radians);
        double s = sin(radians);
        *this *= arma::mat44{1,  0, 0, 0,
                             0,  c, s, 0,
                             0, -s, c, 0,
                             0,  0, 0, 1};
        return *this;
    }

    Transform& Transform::rotateY(double radians) {
        double c = cos(radians);
        double s = sin(radians);
        *this *= arma::mat44{c, 0, -s, 0,
                             0, 1,  0, 0,
                             s, 0,  c, 0,
                             0, 0,  0, 1};
        return *this;
    }

    Transform& Transform::rotateZ(double radians) {
        double c = cos(radians);
        double s = sin(radians);
        *this *= arma::mat44{ c, s, 0, 0,
                             -s, c, 0, 0,
                              0, 0, 1, 0,
                              0, 0, 0, 1};
        return *this;
    }

    Transform& Transform::worldToLocal(const Transform& local) {
        *this = local.i() * (*this);
        return *this;
    }

    Transform& Transform::localToWorld(const Transform& local) {
        *this = local * (*this);
        return *this;
    }

}
}
