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

#include "SE2.h"

#include <nuclear>

#include "utility/math/angle.h"

namespace utility {
namespace math {

    using matrix::Transform;
    using utility::math::angle::normalizeAngle;

    SE2 SE2::localToWorld(const SE2& reference) const {
        double cosAngle = std::cos(angle());
        double sinAngle = std::sin(angle());
        // translates to this + rotZ(this.angle) * reference
        return {
            x() + cosAngle * reference.x() - sinAngle * reference.y(),
            y() + sinAngle * reference.x() + cosAngle * reference.y(),
            angle() + reference.angle() // do not use normalizeAngle here, causes bad things when turning! TODO: unsure on cause
        };
    }

    SE2 SE2::worldToLocal(const SE2& reference) const {
        double cosAngle = std::cos(angle());
        double sinAngle = std::sin(angle());
        SE2 diff = reference - *this;
        // translates to rotZ(this.angle) * (reference - this)
        return {
            cosAngle * diff.x() + sinAngle * diff.y(),
            -sinAngle * diff.x() + cosAngle * diff.y(),
            normalizeAngle(diff.angle())
        };
    }

    SE2 SE2::interpolate(double t, const SE2& target) const {
        SE2 result = *this + t * (target - *this);
        result[2] = normalizeAngle(result.angle());
        return result;
    }

    SE2::operator Transform() const {
        return Transform().translate({x(), y(), 0}).rotateZ(angle());
    }

}
}
