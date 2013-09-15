/*
 * This file is part of NUBots Utility.
 *
 * NUBots Utility is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * NUBots Utility is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with NUBots Utility.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#ifndef UTILITY_MATH_ANGLE_H
#define UTILITY_MATH_ANGLE_H

#include <cmath>

namespace utility {
namespace math {
namespace angle {
    inline double normalizeAngle(const double value) {

        double angle = fmod(value, 2 * M_PI);
        if (angle < -M_PI) angle += 2 * M_PI;
        else if (angle >= M_PI) angle -= 2 * M_PI;

        return angle;
    }

    inline double difference(const double a, const double b) {
        return fmod(((a - b) + M_PI), 2 * M_PI) - M_PI;
    }
}
}
}
#endif
